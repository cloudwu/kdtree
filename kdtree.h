#ifndef KDTREE_H
#define KDTREE_H

#define KDTREE_MAXDEPTH 10

struct boundingbox {
	float minp[3];
	float maxp[3];
};

#define KDTREE_NODES ((1<<KDTREE_MAXDEPTH)-1)

struct kdtree {
	int depth;
	float split[KDTREE_NODES];
	struct boundingbox box[KDTREE_NODES];
};

void kdtree_build(struct kdtree *space, int n, struct boundingbox *aabbs);
int kdtree_insert(struct kdtree *space, const struct boundingbox *aabb, int index);
const struct boundingbox * kdtree_box(struct kdtree *space, int index);
void kdtree_range(struct kdtree *space, int index, int *from_index, int *to_index);

#endif
