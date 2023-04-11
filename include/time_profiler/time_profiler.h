#ifndef TIME_PROFILER_H_
#define TIME_PROFILER_H_

// Activate profiling if the user did not explicitly deactivate it.
#ifndef USE_PROFILER
#define USE_PROFILER 1
#endif

#if USE_PROFILER
// Define the placeholders for setting checkpoints.
#define PROFILER_HOOK() ::time_profiler::TimeProfiler::tick(__FILE__, __LINE__, __FUNCTION__);
// #define xxx ::time_profiler::TimeProfiler::tick(__FILE__, __LINE__, __FUNCTION__);
// #define ___ ::time_profiler::TimeProfiler::tick(__FILE__, __LINE__, __FUNCTION__);
#else
#define PROFILER_HOOK()
#endif

#include <deque>
#include <map>
#include <list>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <functional>
#include <cmath>
#include <iomanip>

namespace time_profiler
{
    /// Checkpoint used for measuring execution time.
    /// Objects of this class store all information necessary to identify
    /// a checkpoint:
    ///  * the file where the checkpoint resides,
    ///  * the line where the checkpoint resides,
    ///  * the point in time when the checkpoint was hit.
    class Checkpoint
    {
    protected:
        /// Name of the file the checkpoint resides in.
        std::string file_;

        /// Number of the line the checkpoint resides in.
        int line_;

        /// Name of the function the checkpoint resides in.
        std::string function_;

        /// Time stamp of the checkpoint.
        std::chrono::system_clock::time_point time_point_;

    public:
        /// Constructor.
        /// Initializes the member variables.
        Checkpoint(const std::string &file, int line, const std::string &function)
            : file_(file),
              line_(line),
              function_(function),
              time_point_(std::chrono::system_clock::now())
        {
        }

        /// Compares the file and line of two checkpoints.
        bool operator==(const Checkpoint &rhs) const
        {
            return (file_ == rhs.file_ && line_ == rhs.line_);
        }

        /// Returns a string that contains the file and line of the checkpoint.
        /// Format: \c <file>:<line>.
        std::string get_file_line() const
        {
            std::stringstream stream;
            stream << file_ << ":" << line_;
            return stream.str();
        }

        /// Get the time the checkpoint was captured.
        std::chrono::system_clock::time_point get_time_point() const
        {
            return time_point_;
        }

        /// Get the file the checkpoint resides in.
        std::string get_file() const
        {
            return file_;
        }

        /// Get the line the checkpoint resides in.
        int get_line() const
        {
            return line_;
        }

        /// Get the function the checkpoint resides in.
        std::string get_function() const
        {
            return function_;
        }
    };

    /// Measurement of the execution time that passed between two checkpoints.
    class SingleMeasurement
    {
    protected:
        /// Start of the measurement.
        Checkpoint start_checkpoint_;

        /// End of the measurement.
        Checkpoint end_checkpoint_;

    public:
        /// Constructor.
        SingleMeasurement(const Checkpoint &start_checkpoint,
                          const Checkpoint &end_checkpoint)
            : start_checkpoint_(start_checkpoint),
              end_checkpoint_(end_checkpoint)
        {
        }

        /// Returns a hash value that is generated from the file and line values
        /// of both the start and the end measurement.
        /// Thus, this hash uniquely identifies every combination
        /// of two checkpoints.
        std::size_t get_hash() const
        {
            std::hash<std::string> hash;
            return hash(start_checkpoint_.get_file_line() + end_checkpoint_.get_file_line());
        }

        /// Get the time that expired between the start checkpoint and the
        /// end checkpoint.
        /// Unit: [us].
        std::chrono::microseconds get_duration() const
        {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                end_checkpoint_.get_time_point() - start_checkpoint_.get_time_point());
        }

        /// Returns the checkpoint where the measurement of execution time started.
        Checkpoint get_start_checkpoint() const
        {
            return start_checkpoint_;
        }

        /// Returns the checkpoint where the measurement of execution time ended.
        Checkpoint get_end_checkpoint() const
        {
            return end_checkpoint_;
        }
    };

    /// Saves the statistics of multiple execution time measurements that have the
    /// same start checkpoint and the same end checkpoint.
    class MultiMeasurement
    {
    protected:
        /// Hash that uniquely identifies the combination of start checkpoint and
        /// end checkpoint.
        std::size_t hash_;

        /// Name of the file where the start checkpoint resides.
        std::string start_file_;

        /// Name of the function where the start checkpoint resides.
        std::string start_function_;

        /// Number of the line where the start checkpoint resides.
        int start_line_;

        /// Name of the file where the end checkpoint resides.
        std::string end_file_;

        /// Name of the function where the end checkpoint resides.
        std::string end_function_;

        /// Number of the line where the end checkpoint resides.
        int end_line_;

        /// Number of single measurements collected by this object.
        int count_;

        /// Sum of the durations of all measurements collected by this object.
        std::chrono::microseconds overall_duration_;

        /// Percentage of consumed time, need to be populated before print
        double percent_;

    public:
        /// Default constructor.
        /// Initializes the member variables to 0.
        MultiMeasurement()
            : hash_(0),
              start_line_(0),
              end_line_(0),
              count_(0),
              overall_duration_(0),
              percent_(0.0)
        {
        }

        /// Collects a measurement.
        /// \return \c true if the hash of the measurement matches the hash of
        /// the measurements collected so far.
        bool add(const SingleMeasurement &measurement)
        {
            // If no measurement has been collected so far, define the member
            // variables.
            if (hash_ == 0)
            {
                hash_ = measurement.get_hash();
                start_file_ = measurement.get_start_checkpoint().get_file();
                start_function_ = measurement.get_start_checkpoint().get_function();
                start_line_ = measurement.get_start_checkpoint().get_line();
                end_file_ = measurement.get_end_checkpoint().get_file();
                end_function_ = measurement.get_end_checkpoint().get_function();
                end_line_ = measurement.get_end_checkpoint().get_line();
            }

            // Check if the measurement starts and ends at the same checkpoints
            // as the other measurements that have been collected.
            if (hash_ != measurement.get_hash())
                return false;

            // Update the statistics.
            count_++;
            overall_duration_ += measurement.get_duration();

            return true;
        }

        /// Returns the number of measurements.
        int count() const
        {
            return count_;
        }

        /// Returns the overall duration of all measurements.
        /// Unit: [us].
        std::chrono::microseconds get_overall_duration() const
        {
            return overall_duration_;
        }

        /// Computes the average duration of all measurements.
        /// Unit: [us].
        std::chrono::microseconds get_average_duration() const
        {
            std::chrono::microseconds average_duration(overall_duration_);

            if (count_ > 0)
                average_duration /= count_;

            return average_duration;
        }

        /// Compares measurements based on their overall time consumption.
        bool operator<(const MultiMeasurement &rhs) const
        {
            return get_overall_duration() < rhs.get_overall_duration();
        }

        /// Returns the hash of the measurements collected so far.
        /// If no measurements have been collected, 0 is returned.
        std::size_t get_hash() const
        {
            return hash_;
        }

        /// Returns the file where the start checkpoint resides.
        std::string get_start_file() const
        {
            return start_file_;
        }

        /// Returns the function where the start checkpoint resides.
        std::string get_start_function() const
        {
            return start_function_;
        }

        /// Returns the number of the line where the start checkpoint resides.
        int get_start_line() const
        {
            return start_line_;
        }

        /// Returns the file where the end checkpoint resides.
        std::string get_end_file() const
        {
            return end_file_;
        }

        /// Returns the function where the end checkpoint resides.
        std::string get_end_function() const
        {
            return end_function_;
        }

        /// Returns the number of the line where the end checkpoint resides.
        int get_end_line() const
        {
            return end_line_;
        }

        double get_percent() const
        {
            return percent_;
        }

        void set_percent(double p)
        {
            percent_ = p;
        }
    };

    /// Prints the statistics of the given measurements to the console.
    class Printer
    {
    private:
        /// Measurements whose statistics to print.
        std::vector<MultiMeasurement> measurements_;

        /// Width of the output lines.
        static const int line_width = 131;

        /// Width of the column indicating the file of a checkpoint.
        static const int file_col_width = 30;

        /// Width of the column indicating the function of a checkpoint.
        static const int function_col_width = 40;

        /// Width of the column indicating the line of a checkpoint.
        static const int line_col_width = 5;

        /// Width of the column indicating the count of a measurement.
        static const int count_col_width = 10;

        /// Width of the column indicating the average duration of a measurement.
        static const int avg_duration_col_width = 15;

        /// Width of the column indicating the overall duration of a measurement.
        static const int ovr_duration_col_width = 15;

        /// Width of the column indicating the overall duration of a measurement.
        static const int ovr_percentage_col_width = 10;

    public:
        /// Adds a measurement whose statistics will be printed when print()
        /// is called.
        void add(const MultiMeasurement &measurement)
        {
            measurements_.push_back(measurement);
        }

        /// Adds a list of measurements whose statistics will be printed when
        /// print() is called.
        void add(const std::list<MultiMeasurement> &measurements)
        {
            std::list<MultiMeasurement>::const_iterator lit;
            for (lit = measurements.begin(); lit != measurements.end(); lit++)
                add(*lit);
        }

        /// Prints the statistics of the given measurements.
        void print() const
        {
            std::cerr << create_table();
        }

        /// Creates a table that shows the statistics of the given measurements.
        std::string create_table() const
        {
            // If no measurements are given, return an empty string.
            if (measurements_.size() <= 0)
                return std::string();

            // Create the header of the table.
            std::stringstream stream;
            // stream << create_title();
            stream << create_header();

            // Add each measurement to the table.
            for (int i = 0; i < (int)measurements_.size(); i++)
            {
                stream << create_entry(measurements_[i]);
                stream << create_hline((i < (int)measurements_.size() - 1) ? '-' : '=');
            }

            return stream.str();
        }

        /// Saves the current profiling information to \c $HOME/.TimeProfiler/log.
        void save_log() const
        {
            // If no measurements are given, abort.
            if (measurements_.size() <= 0)
                return;

            // Create the folder name.
            std::stringstream folder_name;
            folder_name << getenv("HOME") << "/.TimeProfiler/log";
            std::filesystem::path folder_path(folder_name.str());

            // Create the folder.
            std::filesystem::create_directories(folder_path);

            // Create the name of the log file from the current date and time.
            std::stringstream file_name;
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            file_name << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
            file_name << ".log";

            // Save the log file.
            std::ofstream logfile;
            logfile.open(std::string(
                             std::filesystem::canonical(folder_path).string() + "/" + file_name.str())
                             .c_str());
            logfile << create_table();
            logfile.close();
        }

    protected:
        /// Generates a string containing a horizontal line
        /// consisting of the given character.
        static std::string create_hline(char fill = '#')
        {
            std::stringstream stream;
            stream << std::setfill(fill) << std::setw(line_width) << fill
                   << std::endl;
            return stream.str();
        }

        /// Generates a string containing the profiler's heading.
        static std::string create_title()
        {
            const std::string title(" PROFILED WITH TimeProfiler ");

            const char fill = '#';
            std::stringstream stream;
            stream << std::setfill(fill)
                   << std::setw(line_width) << fill
                   << std::endl
                   << std::setw((line_width - title.length()) / 2) << fill << title
                   << (title.length() % 2 > 0 ? std::string(1, fill) : "")
                   << std::setw((line_width - title.length()) / 2) << fill
                   << std::endl
                   << std::setw(line_width) << fill
                   << std::endl;

            return stream.str();
        }

        /// Generates a string with the headers of all columns.
        static std::string create_header()
        {
            std::stringstream stream;
            stream << create_hline('=')
                   << std::setfill(' ')
                   << std::setw(file_col_width) << std::left << "File"
                   << "|" << std::setw(function_col_width) << std::left << "Function"
                   << "|" << std::setw(line_col_width) << std::right << "Line "
                   << "|" << std::setw(count_col_width) << std::right << "Count "
                   << "|" << std::setw(avg_duration_col_width) << std::right << "Average [us] "
                   << "|" << std::setw(ovr_duration_col_width) << std::right << "Overall [us]"
                   << "|" << std::setw(ovr_percentage_col_width) << std::right << "Percent %"
                   << std::endl
                   << create_hline('=');

            return stream.str();
        }

        /// Generates a table entry for the given measurement.
        static std::string create_entry(const MultiMeasurement &measurement)
        {
            // Create a line indicating where the measurement started.
            const std::string file_start(crop_path(measurement.get_start_file()));
            std::stringstream stream;
            stream << std::setfill(' ')
                   << std::setw(file_col_width) << std::left << file_start
                   << "|" << std::setw(function_col_width) << std::left << measurement.get_start_function()
                   << "|" << std::setw(line_col_width) << std::right << measurement.get_start_line()
                   << "|" << std::setw(count_col_width) << std::right << " "
                   << "|" << std::setw(avg_duration_col_width) << std::right << " "
                   << "|" << std::endl;

            // Show the file name in the second line only if it
            // is a different file.
            std::string file_end(crop_path(measurement.get_end_file()));
            if (file_start == file_end)
                file_end.clear();

            // Create a line indicating where the measurement ended
            // and how long it took.
            stream << std::setfill(' ')
                   << std::setw(file_col_width) << std::left
                   << file_end << "|"
                   << std::setw(function_col_width) << std::left
                   << measurement.get_end_function() << "|"
                   << std::setw(line_col_width) << std::right
                   << measurement.get_end_line() << "|"
                   << std::setw(count_col_width) << std::right
                   << insert_separators(measurement.count()) << "|"
                   << std::setw(avg_duration_col_width) << std::right
                   << insert_separators(measurement.get_average_duration().count()) << "|"
                   << std::setw(ovr_duration_col_width) << std::right
                   << insert_separators(measurement.get_overall_duration().count()) << "|"
                   << std::setw(ovr_percentage_col_width) << std::right
                   << std::setprecision(5) << ((measurement.get_percent() > 0.000001) ? (measurement.get_percent() * 100) : 0.0)
                   << std::endl;

            return stream.str();
        }

        /// Cuts the given file path after the last slash and returns the file name.
        static std::string crop_path(const std::string &file_name)
        {
            std::size_t slash_position = file_name.find_last_of("/\\");
            return file_name.substr(slash_position + 1);
        }

        /// Inserts thousands separators into the given number.
        static std::string insert_separators(long int n)
        {
            std::stringstream stream;
            stream << n;
            const std::string n_str = stream.str();

            stream.str("");
            for (int pos = 0; pos < (int)n_str.length(); pos++)
            {
                stream << n_str.at(pos);

                if ((n_str.length() - 1 - pos) >= 3 && (n_str.length() - 1 - pos) % 3 == 0)
                    stream << ",";
            }

            return stream.str();
        }

        /// If the given number would exceed the given number of digits to display,
        /// crop it.
        /// Example: crop_number(12345, 4) returns ">999".
        static std::string crop_number(long int number, int n_digits)
        {
            // Make sure n_digits is in the valid range.
            n_digits = std::max(2, n_digits);

            // Compute the minimum and maximum numbers which can be displayed
            // with the given number of digits.
            long int max_number = std::pow(10, n_digits - 1) - 1;
            long int min_number = -max_number;

            // Crop the number, if necessary.
            std::stringstream stream;
            if (number <= max_number && number >= min_number)
                stream << number;
            if (number > max_number)
                stream << ">" << max_number;
            if (number < min_number)
                stream << "<" << min_number;

            return stream.str();
        }
    };

    /// Simple CPU execution time profiler.
    ///
    /// Measurement points are added by inserting three underscores \c ___ or three X \c xxx in the code.
    /// The profiler prints its statistics on the console when being destroyed.
    /// The statistics can also be printed by calling TimeProfiler::print_statistics().
    ///
    /// \note Profiling is enabled by default. It can be globally disabled
    /// by simply adding
    /// \code
    /// #define USE_PROFILER 0
    /// \endcode
    /// to the source file before including \c TimeProfiler.h.
    /// \code
    /// #define USE_PROFILER 1
    /// \endcode enables profiling again.
    ///
    /// \note This class is not thread-safe. It is designed to be used with
    /// single-threaded programs.
    class TimeProfiler
    {
    private:
        std::deque<Checkpoint> checkpoints_;
        std::map<std::size_t, MultiMeasurement> measurement_map_;

    private:
        /// Default constructor.
        /// Inaccessible from outside the class.
        TimeProfiler()
        {
        }

        /// Destructor.
        /// Computes and prints the collected statistics and saves them to a
        /// log file: \c $HOME/.TimeProfiler/log.
        ~TimeProfiler()
        {
            print_statistics();
            save_log();
        }

        /// Copy constructor.
        /// Inaccessible from outside the class.
        TimeProfiler(const TimeProfiler &TimeProfiler);

        /// Assignment operator.
        /// Inaccessible from outside the class.
        TimeProfiler &operator=(const TimeProfiler &TimeProfiler);

        /// Returns the singleton instance of the profiler.
        static TimeProfiler &get_instance()
        {
            // Create the single instance of this class.
            static TimeProfiler TimeProfiler;
            return TimeProfiler;
        }

        /// Returns a sorted list of the measurements that were made.
        /// The measurements are sorted with respect to the overall execution time
        /// in descending order.
        static std::list<MultiMeasurement> sort_measurements()
        {
            // Copy the elements of the measurement map into a list
            // that can be sorted.
            std::list<MultiMeasurement> measurement_list;
            std::map<std::size_t, MultiMeasurement> &measurement_map = get_instance().measurement_map_;
            std::map<std::size_t, MultiMeasurement>::const_iterator mit;
            std::chrono::microseconds total_duration(0);
            for (mit = measurement_map.begin();
                 mit != measurement_map.end();
                 mit++)
            {
                measurement_list.push_back(mit->second);
                total_duration += +mit->second.get_overall_duration();
            }

            // Update execution time percentage
            double total_microseconds = total_duration.count();
            for (auto &measurement : measurement_list)
            {
                measurement.set_percent(measurement.get_overall_duration().count() / total_microseconds);
            }

            // Sort the measurements based on their overall execution times,
            // starting with the largest value.
            measurement_list.sort();
            measurement_list.reverse();

            return measurement_list;
        }

    public:
        /// Adds a measurement.
        static void tick(
            const std::string &file, int line, const std::string &function)
        {

#if USE_PROFILER
            // Add the measurement point.
            std::deque<Checkpoint> &checkpoints = get_instance().checkpoints_;
            checkpoints.push_back(Checkpoint(file, line, function));

            if (checkpoints.size() >= 2)
            {
                SingleMeasurement measurement(checkpoints[0], checkpoints[1]);
                get_instance().measurement_map_[measurement.get_hash()].add(
                    measurement);
                checkpoints.pop_front();
            }
#endif
        }

        /// Prints the statistics.
        static void print_statistics()
        {
#if USE_PROFILER
            // Print the sorted list of all measurements.
            Printer printer;
            printer.add(sort_measurements());

            printer.print();
#endif
        }

        /// Saves a log file with the statistics under \c $HOME/.TimeProfiler/log.
        static void save_log()
        {
#if USE_PROFILER
            Printer printer;
            printer.add(sort_measurements());

            printer.save_log();
#endif
        }
    };

} // namespace time_profiler
#endif // #define TIME_PROFILER_H_