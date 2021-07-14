#ifndef BWI_PERCEPTION_COMPARISON_H
#define BWI_PERCEPTION_COMPARISON_H

namespace bwi_perception {
    template<typename T, typename U>
    bool compare_by_second(const std::pair<T, U> &lhs, const std::pair<T, U> &rhs) {
        return lhs.second > rhs.second;
    }

    template<typename T, typename U>
    bool compare_by_first(const std::pair<T, U> &lhs, const std::pair<T, U> &rhs) {
        return lhs.first > rhs.first;
    }

    bool compare_cluster_size(const pcl::PointIndices &lhs, const pcl::PointIndices &rhs) {
        return lhs.indices.size() < rhs.indices.size();
    }
}
#endif //BWI_PERCEPTION_COMPARISON_H
