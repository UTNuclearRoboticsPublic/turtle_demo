#include <vector>
#include <stdexcept>


using std::size_t;

class DecisionModule {
    public:
        DecisionModule(size_t input_size, size_t output_size);
        const std::vector<float> getUtilities(const std::vector<float> input_data) const;
    private:
        size_t input_size_;
        size_t output_size_;
};