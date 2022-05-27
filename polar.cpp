//
// polar.cpp
//
// capture a user selectable length of data, mark in angles for output
//

#include <iostream>
#include <string>
#include <string_view>
#include <array>
#include <vector>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <numbers>

#include "gpio_help.h"

using namespace std::literals;
constexpr double gain { 10.0 };

void welcome()
{
    std::cerr << "polar 0.01\n";
    std::cerr << "Copyright (c) 2022 paul@epicyclism.com\n\n";
    std::cerr << "Capture arbitrary length\n";
}

void usage()
{
    std::cerr << "Usage: polar <revs to capture> > output.dat\n";
    std::cerr << "writes captured data to cout\n";
}

std::vector<float> capture( gpiod::line& line, int revs)
{
	std::vector<int64_t> data ( revs * pts_per_rev );
	auto dd = data.begin();
	while(dd != data.end())
	{
		if(line.event_wait(1000ms))
		{
			auto e = line.event_read();
			*dd = e.timestamp.count();
			++dd;
		}
		else
			break;
	}
    // calculate the rpm
    auto duration = data.back() - data.front();
    auto rpm = double(60000000000) * (data.size() - 1) / (double(duration) * pts_per_rev);
    std::cout << "# rpm = " << rpm << "\n";

    // process to intervals
    std::adjacent_difference(data.begin(), data.end(), data.begin());
	data.erase(data.begin());
   	std::vector<float> velocities (data.size());

	auto factor = double(60000000000) / pts_per_rev;
	std::transform(data.begin(), data.end(), velocities.begin(), [=](auto i){ return factor / i ; });
	// amplify the variation by subracting rpm, multiplying, and putting rpm back
	std::transform(velocities.begin(), velocities.end(), velocities.begin(), [rpm](auto v){ return (v - rpm) * gain + rpm;});

    return velocities;
}

void write_buf(std::vector<float> const& buf)
{
    double angle { 0.0 };
    double angle_inc {std::numbers::pi * 2.0 / pts_per_rev};

	std::for_each(buf.begin(), buf.end(), [&angle, angle_inc](auto v){ std::cout << angle << " " << v << "\n"; angle += angle_inc;});
}

int main(int ac, char** av)
{
    welcome();
    if( ac == 1 || av[1][0] == '-' || av[1][0] == '/')
    {
        usage();
        return 1;
    }
    auto revs = ::atoi(av[1]);
    if(revs == 0)
    {
        usage();
        return 1;
    }
   	auto lines = initialise<gpiod::line_request::EVENT_RISING_EDGE>(line_q);
    if(lines.empty())
    {
        std::cout << "Failed to intiialise GPIO!\n";
        return 1;
    }
    std::cerr << "Starting capture...\n";
    auto buf = capture(lines[0], revs);
    write_buf(buf);
}