#ifndef MDC_ENCODER_HPP
#define MDC_ENCODER_HPP

class MDCEncoder
{
    public:
        // Constructor initialising member variables
        MDCEncoder() : delta_count(0), last_count(0), is_reversed(false) {}

        // Initialise encoder count
        void initCount(int init_count)
        {
            // Change in encoder count (always starts = 0)
            delta_count = 0;
            // Previously recorded count
            last_count = init_count;
        }

        // Updates encoder count based on newly received count data
        void updateCount(int new_count)
        {
            int increment = new_count - last_count;

            // Increase delta_count by increment
            delta_count += increment;
            // Set last_count equal to the new_count value just processed
            last_count = new_count;
        }

        void setReversed(bool reversed)
        {
            is_reversed = reversed;
        }

        // Get delta value
        int getDelta()
        {
            // Uses delta_value to ensure correct accumulated delta_count is returned
            int delta_value = delta_count;
            // Resets delta_count to zero to immediately restart accumulating pulses
            delta_count = 0;
            // Performs negation if encoder is set as reversed 
            if (is_reversed)
            {
                return -delta_value;
            }
            else
            {
                return delta_value;
            }
        }

    private:
        // Define encoder parameters member variables
        int delta_count, last_count; // delta_count () and last_count () encoder counts
        bool is_reversed; // Indicates whether encoder is reversed or not
};

#endif