#include <list>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include "PointToVoxel.hpp"

namespace cloud{
  using Voxel = Eigen::Vector3i;


  class LFUCache{
    public:
      LFUCache(int max_size):
        max_size_(max_size),
        cache_map_(max_size),
        cache_list_(max_size){}

      inline void put(const Voxel& key){
        auto it = cache_map_.find(key);
        cache_list_.push_front(key);
        if (it != cache_map_.end()){
          it->second = it->second + 1;

        } else{
          cache_map_.emplace(key,1);
        }
        if (cache_list_.size() > max_size_){
          auto last =  cache_list_.end();
          last--;
          auto map_it = cache_map_.find(*last);
          map_it->second = map_it->second - 1;
          if (map_it->second == 0){
            cache_map_.erase(*last);
          }
          cache_list_.pop_back();
        }
      }

      inline int get(const Voxel& key){
        auto it = cache_map_.find(key);
        return it != cache_map_.end() ? it->second : -1;
      }

      void reset(){
        cache_list_.clear();
        cache_list_.resize(max_size_);
        cache_map_.clear();
      }
    private:
      size_t max_size_;
      std::list<Voxel> cache_list_;
      std::unordered_map<Voxel,int> cache_map_;
};

}// namespace cloud
