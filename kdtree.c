#include "kdtree.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DIRECTION_X 0
#define DIRECTION_Y 1
#define DIRECTION_Z 2

static inline float
center(const struct boundingbox *aabb, int index) {
	return (aabb->minp[index] + aabb->maxp[index]) / 2;
}

static void
partition_(struct boundingbox *aabb, int n, int part, int index, float pivot) {
	for (;;) {
		int small_index = 0;
		int big_index = n-1;
		float small_sum = 0;
		float big_sum = 0;
		while (small_index<=big_index) {
			float t = center(aabb+small_index, index);
			if (t <= pivot) {
				small_sum += t;
				++small_index;
			} else {
				big_sum += t;
				struct boundingbox tmp = aabb[small_index];
				aabb[small_index] = aabb[big_index];
				aabb[big_index] = tmp;
				--big_index;
			}
		}
		if (small_index == part) {
			return;
		}
		if (small_index < part) {
			aabb += small_index;
			n -= small_index;
			part -= small_index;
			pivot = big_sum / (n - small_index);
		} else {
			n = small_index;
			pivot = small_sum / small_index;
		}
	}
}

static float
split_box(const struct boundingbox *bb, int *direction) {
	float x = bb->maxp[0] - bb->minp[0];
	float y = bb->maxp[1] - bb->minp[1];
	float z = bb->maxp[2] - bb->minp[2];

	if (x >= y && x >= z) {
		*direction = DIRECTION_X;
		return (bb->maxp[0] + bb->minp[0])/2;
	} else if (y > x && y > z) {
		*direction = DIRECTION_Y;
		return (bb->maxp[1] + bb->minp[1])/2;
	} else {
		*direction = DIRECTION_Z;
		return (bb->maxp[2] + bb->minp[2])/2;
	}
}

static int
partition(struct boundingbox *aabb, int n, const struct boundingbox *bb) {
	int part = n/2;
	int direction;
	float split = split_box(bb, &direction);
	partition_(aabb, n, part, direction, split);
	return direction;
}

static void
merge_n(struct boundingbox *v1, int n, const struct boundingbox *aabb) {
	int i,j;
	*v1 = aabb[0];
	for (j=1;j<n;j++) {
		const struct boundingbox *v2 = aabb+j;
		for (i=0;i<3;i++) {
			if (v2->minp[i] < v1->minp[i]) {
				v1->minp[i] = v2->minp[i];
			}
			if (v2->maxp[i] > v1->maxp[i]) {
				v1->maxp[i] = v2->maxp[i];
			}
		}
	}
}

static int 
merge(struct boundingbox *v1, const struct boundingbox *v2) {
	if (v1->minp[0] == 0 && v1->maxp[0] == 0) {
		*v1 = *v2;
		return 1;
	}
	int change = 0;
	int i;
	for (i=0;i<3;i++) {
		if (v2->minp[i] < v1->minp[i]) {
			v1->minp[i] = v2->minp[i];
			change = 1;
		}
		if (v2->maxp[i] > v1->maxp[i]) {
			v1->maxp[i] = v2->maxp[i];
			change = 1;
		}
	}
	return change;
}

static inline int
get_direction(float v) {
	union {
		float f;
		uint32_t n;
	} u;
	u.f = v;
	return u.n & 3;
}

static inline float
combine_direction(float v, int d) {
	union {
		float f;
		uint32_t n;
	} u;
	u.f = v;
	u.n = (u.n & ~3) | d;
	return u.f;
}

static void
split_boundingbox(struct kdtree *space, int slot, int maxslots) {
	int direction;
	float split = split_box(&space->box[slot], &direction);
	space->split[slot] = combine_direction(split, direction);
	int left = slot * 2 + 1;
	if (left >= maxslots)
		return;
	space->box[left] = space->box[slot];
	space->box[left].maxp[direction] = split;
	split_boundingbox(space, left, maxslots);

	int right = left + 1;
	space->box[right] = space->box[slot];
	space->box[right].minp[direction] = split;
	split_boundingbox(space, right, maxslots);
}

static void
calc_boundingbox(struct kdtree *space, int slot, int n, struct boundingbox *aabbs, int maxslots) {
	merge_n(&space->box[slot], n, aabbs);
	if (n > 1) {
		int direction = partition(aabbs, n, &space->box[slot]);
		int left_n = n / 2;
		float split = (center(&aabbs[left_n], direction) + center(&aabbs[left_n+1], direction))/2;
		space->split[slot] = combine_direction(split, direction);
		// todo
		int left = slot * 2 + 1;
		if (left >= maxslots)
			return;
		calc_boundingbox(space, left, left_n, aabbs, maxslots);
		calc_boundingbox(space, left+1, n - left_n, aabbs+left_n, maxslots);
	} else {
		split_boundingbox(space, slot, maxslots);
	}
}

void
kdtree_build(struct kdtree *space, int n, struct boundingbox *aabbs) {
	int depth = 1;
	int t = n;
	while (depth < KDTREE_MAXDEPTH && t > 0) {
		depth++;
		t = t / 2;
	}
	space->depth = depth;

	int maxslots = (1 << depth) - 1;

	if (n > 0)
		calc_boundingbox(space, 0, n, aabbs, maxslots);
	memset(space->box, 0, maxslots * sizeof(struct boundingbox));
}

int
kdtree_insert(struct kdtree *space, const struct boundingbox *aabb) {
	int maxslots = (1 << space->depth) - 1;
	int slot = 0;
	while (slot * 2 < maxslots) {
		float split = space->split[slot];
		int direction = get_direction(split);
		float c = center(aabb, direction);
		if (c <= split) {
			slot = slot * 2 + 1;
		} else {
			slot = slot * 2 + 2;
		}
	}
	int s = slot; 
	for (;;) {
		if (!merge(&space->box[s], aabb))
			break;
		if (s == 0)
			break;
		s = (s - 1) / 2;
	}
	return slot;
}

static void
print_aabb(const struct boundingbox *bb) {
	printf("(%f %f %f)-(%f %f %f)\n",
		bb->minp[0],bb->minp[1],bb->minp[2],
		bb->maxp[0],bb->maxp[1],bb->maxp[2]);
}

int main() {
	struct boundingbox bb[8] = {
		{ {3,3,3}, {4,4,4}, },
		{ {2,2,2}, {3,3,3}, },
		{ {7,7,7}, {8,8,8}, },
		{ {1,1,1}, {2,2,2}, },
		{ {5,5,5}, {6,6,6}, },
		{ {0,0,0}, {1,1,1}, },
		{ {4,4,4}, {5,5,5}, },
		{ {6,6,6}, {7,7,7}, },
	};

	struct kdtree space;

	kdtree_build(&space, 8, bb);

	int slots = (1 << space.depth) - 1;
	int i;
	for (i=0;i<slots;i++) {
		float v = space.split[i];
		printf("%d: %d %f\n", i, get_direction(v), v);
	}

	struct boundingbox tmp = {
		{ 1,1,1 }, {2,2,2},
	};
	printf("Insert %d\n", kdtree_insert(&space, &tmp));

	for (i=0;i<slots;i++) {
		print_aabb(&space.box[i]);
	}
	return 0;
}
