#include <vector>
#include <stdexcept>


using std::size_t;

namespace DS {

/**
 * @brief The DecisionModule receives data from the DynamicSelector and returns utility values
 * corresponding to the Selector's children.
 * 
 * This class is abstract. To use it, create a derived class that overrides the pure
 * virtual method computeUtilities() with a function that returns a vector of utility values.
 */
class DecisionModule {
    public:
        DecisionModule(size_t input_size, size_t output_size);
        const std::vector<float> getUtilities(const std::vector<float> input_data) const;

    protected:
        size_t input_size_;
        size_t output_size_;

    private:
        virtual const std::vector<float> computeUtilities(const std::vector<float> input_data) const = 0;
};
} // DS