#include <vector>
#include <pair>

template<typename ITEM>
class VoxelHashing
{
    public:
    using entry_t = std::pair<size_t, ITEM>;
    using bucket_t = std::vector<entry_t>;

    constexpr size_t DEFAULT_BUCKET_SIZE = 100;
    constexpr size_t DEFAULT_NUMBER_OF_BUCKETS = 100;
    VoxelHashing(size_t buckets = DEFAULT_NUMBER_OF_BUCKETS, size_t reserved_items_per_bucket = DEFAULT_BUCKET_SIZE)
    {
        m_pool = std::vector<bucket_t>(buckets);
        for(auto& bucket: m_pool)
        {
            bucket.reserve(reserved_items_per_bucket);
        }
    }
    inline void insert(size_t hash, const ITEM value)
    {
        size_t bucketID = hash % m_pool.size;
        m_pool[bucketID].push_back(std::make_pair(hash,value));
    }
    inline const ITEM* at(size_t hash) const
    {
        size_t bucketID = hash % m_pool.size;
        for(auto& entry: m_pool[bucketID])
        {
            if(entry.first == hash)
                return &entry.second;
        }
    }
    inline void clear()
    {
        for(auto& bucket: m_pool)
        {
            bucket.clear();
        }
    }
    private:
    std::vector<bucket_t> m_pool;
};
