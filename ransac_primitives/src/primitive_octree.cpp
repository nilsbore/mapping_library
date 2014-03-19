#include "primitive_octree.h"

primitive_octree::primitive_octree(double resolution) :
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB, primitive_leaf>(resolution)
{

}

void primitive_octree::setInputCloud(const PointCloudConstPtr &cloud_arg, const IndicesConstPtr &indices_arg)
{
    super::setInputCloud(cloud_arg, indices_arg);
    points_left = input_->size();
}

primitive_octree::~primitive_octree()
{

}

int primitive_octree::size()
{
    return points_left;
}

void primitive_octree::remove_points(const std::vector<int>& inds)
{
    for (const int& ind : inds) {
        remove_point(ind);
        --points_left;
    }
}

void primitive_octree::remove_point(int ind)
{
    pcl::octree::OctreeKey key;
    PointT point = input_->points[ind];

    // generate key for point
    this->genOctreeKeyforPoint(point, key);

    pcl::octree::OctreeNode* result = NULL;
    find_node_recursive(key, depth_mask_, root_node_, result, 0);
    if (result == NULL) {
        return;
    }

    primitive_leaf_node* leafNode = dynamic_cast<primitive_leaf_node*> (result);
    if (leafNode != NULL) {
        if (!leafNode->getContainer().remove_if_equal(ind)) {
            std::cout << "Couldn't remove point!" << std::endl;
            exit(0); // FIXME: this happened once in the big dataset for some reason
        }
    }
    else {
        std::cout << "Leaf is null!" << std::endl;
        exit(0);
    }
}

void primitive_octree::find_points_at_depth(std::vector<DataT>& inds, const PointT& point, int depth)
{
    if (depth == getTreeDepth()) {
        serialize_node_recursive(root_node_, &inds);
        return;
    }

    pcl::octree::OctreeKey key;

    // generate key for point
    this->genOctreeKeyforPoint(point, key);

    pcl::octree::OctreeNode* result = 0;
    find_node_recursive(key, depth_mask_, root_node_, result, depth);

    if (result == NULL) {
        std::cout << "Is null!" << std::endl;
    }

    if (result->getNodeType() == pcl::octree::BRANCH_NODE) {
        serialize_node_recursive(static_cast<const BranchNode*>(result), &inds);
    }
    else if (result->getNodeType() == pcl::octree::LEAF_NODE) {
        const LeafNode* childLeaf = static_cast<const LeafNode*>(result);
        //childLeaf->getData(inds);
        childLeaf->getContainer().getPointIndices(inds);
    }
    else {
        std::cout << "Couldn't find node at required level." << std::endl;
        exit(0);
    }

}

void primitive_octree::find_potential_inliers(std::vector<DataT>& inds, base_primitive* primitive, double margin)
{
    pcl::octree::OctreeKey newKey;
    unsigned int treeDepth_arg = 0;
    serialize_inliers(root_node_, newKey, treeDepth_arg, &inds, primitive, margin);
}

void primitive_octree::serialize_inliers(const BranchNode* branch_arg, pcl::octree::OctreeKey& key_arg,
                                         unsigned int treeDepth_arg, std::vector<DataT>* dataVector_arg,
                                         base_primitive* primitive, double margin) const
{

    // child iterator
    unsigned char childIdx;
    Eigen::Vector3f min_pt;
    Eigen::Vector3f max_pt;
    Eigen::Vector3d mid;
    double l;
    double dist;
    ++treeDepth_arg;

    // iterate over all children
    for (childIdx = 0; childIdx < 8; childIdx++)
    {


        // if child exist
        if (branch_arg->hasChild(childIdx))
        {
            // add current branch voxel to key
            key_arg.pushBranch(childIdx);

            genVoxelBoundsFromOctreeKey(key_arg, treeDepth_arg, min_pt, max_pt);
            l = double(max_pt(0) - min_pt(0));
            mid = 0.5*(min_pt + max_pt).cast<double>();
            //std::cout << "Diff: " << (max_pt - min_pt).transpose() << std::endl;
            dist = primitive->distance_to_pt(mid);
            if (dist > sqrt(3.0)/2.0*l + margin) {
                key_arg.popBranch();
                continue;
            }

            const pcl::octree::OctreeNode *childNode = branch_arg->getChildPtr(childIdx);

            switch (childNode->getNodeType ())
            {
            case pcl::octree::BRANCH_NODE:
            {
                // recursively proceed with indexed child branch
                serialize_inliers(static_cast<const BranchNode*>(childNode), key_arg,
                                  treeDepth_arg, dataVector_arg, primitive, margin);
                break;
            }
            case pcl::octree::LEAF_NODE:
            {
                const LeafNode* childLeaf = static_cast<const LeafNode*>(childNode);

                if (dataVector_arg) {
                    //childLeaf->getData (*dataVector_arg);
                    childLeaf->getContainer().getPointIndices(*dataVector_arg);
                }

                break;
            }
            default:
                break;
            }

            // pop current branch voxel from key
            key_arg.popBranch();
        }
    }
}

void primitive_octree::find_node_recursive(const pcl::octree::OctreeKey& key_arg,
                                           unsigned int depthMask_arg,
                                           BranchNode* branch_arg,
                                           pcl::octree::OctreeNode*& result_arg, int depth) const
{
    // index to branch child
    unsigned char childIdx;

    // find branch child from key
    childIdx = key_arg.getChildIdxWithDepthMask(depthMask_arg);

    pcl::octree::OctreeNode* childNode = (*branch_arg)[childIdx];
    if (childNode) {
        switch (childNode->getNodeType()) {
        case pcl::octree::BRANCH_NODE:
            // we have not reached maximum tree depth
            BranchNode* childBranch;
            childBranch = static_cast<BranchNode*> (childNode);

            if (depthMask_arg == 1 << depth) { // 2^depth
                result_arg = childNode;
            }
            else {
                find_node_recursive(key_arg, depthMask_arg / 2, childBranch, result_arg, depth);
            }
            break;

        case pcl::octree::LEAF_NODE:
            // return existing leaf node
            result_arg = childNode;
            break;
        }
    }

}

void primitive_octree::serialize_node_recursive(const BranchNode* branch_arg, std::vector<DataT>* dataVector_arg) const
{

    // child iterator
    unsigned char childIdx;

    // iterate over all children
    for (childIdx = 0; childIdx < 8; childIdx++)
    {

        // if child exist
        if (branch_arg->hasChild(childIdx))
        {
            // add current branch voxel to key
            //key_arg.pushBranch(childIdx);

            const pcl::octree::OctreeNode *childNode = branch_arg->getChildPtr(childIdx);

            switch (childNode->getNodeType ())
            {
            case pcl::octree::BRANCH_NODE:
            {
                // recursively proceed with indexed child branch
                serialize_node_recursive(static_cast<const BranchNode*>(childNode), dataVector_arg);
                break;
            }
            case pcl::octree::LEAF_NODE:
            {
                const LeafNode* childLeaf = static_cast<const LeafNode*>(childNode);

                if (dataVector_arg) {
                    //childLeaf->getData (*dataVector_arg);
                    childLeaf->getContainer().getPointIndices(*dataVector_arg);
                }

                break;
            }
            default:
                break;
            }

        }
    }
}
