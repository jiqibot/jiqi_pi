#ifndef MDC_ENCODER_HPP
#define MDC_ENCODER_HPP

// Included to allow use of std::string and std::map functions
#include <string>
#include <map>

class MDCEncoder
{
    public:
        // Constructor initialising member variables
        MDCEncoder() : range(0), low_thresh(0), high_thresh(0), delta_count(0), last_count(0), is_reversed(false) {}

        // Set encoder range (and thresholds)
        void setRange(int low, int high)
        {
            // Calculate total range
            range = high - low + 1;
            // Calculate thresholds - value triggering wrapping count
            // ADJUST 30 & 70 BASED ON UPDATE RATE / TESTING
            low_thresh = low + range * 30 / 100;
            high_thresh = low + range * 70 / 100;
        }

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
            int increment;
            // Wraps around upper limit (from +ve to -ve)
            if (last_count > high_thresh && new_count < low_thresh)
            {
                increment = new_count + range - last_count;
            }
            // Wraps around lower limit (from -ve to +ve)
            else if (last_count < low_thresh && new_count > high_thresh)
            {
                increment = new_count - range - last_count;
            }
            // Standard incrementation
            else
            {
                increment = new_count - last_count;
            }

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
            // Resets delta_count to zero to immediately restart accumulating ticks
            delta_count = 0;
            // Performs negation if encoder is set as reversed 
            if (is_reversed)
            {
                return -delta_count;
            }
            else
            {
                return delta_count;
            }
        }

        std::map<std::string, int> getRange()
        {
            return {{"range", range}, {"low_thresh", low_thresh}, {"high_thresh", high_thresh}};
        }

    private:
        // Define encoder parameters member variables
        int range; // Encoder range
        int low_thresh, high_thresh; // Lower and upper encoder thresholds
        int delta_count, last_count; // delta_count () and last_count () encoder counts
        bool is_reversed; // Indicates whether encoder is reversed or not
};

#endif