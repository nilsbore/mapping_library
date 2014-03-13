#include "primitive_leaf.h"

#include <algorithm>

primitive_leaf::primitive_leaf()
{
}

primitive_leaf::primitive_leaf(const OctreeContainerPointIndices &source) : OctreeContainerPointIndices(source)
{

}

bool primitive_leaf::remove_if_equal(int ind)
{
    int sz = leafDataTVector_.size();
    if (sz == 0) {
        return false;
    }
    std::vector<int>::iterator rm = std::remove_if(leafDataTVector_.begin(), leafDataTVector_.end(), [&ind](const int& i) -> bool
    {
        return i == ind;
    });
    leafDataTVector_.erase(rm, leafDataTVector_.end());
    return leafDataTVector_.size() < sz;
}
