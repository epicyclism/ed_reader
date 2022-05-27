//
// tacho.cpp
//
// continuous speed reporting
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
#include <thread>
#include <utility>

#include "gpio_help.h"
using namespace std::literals;

void tacho(std::stop_token stop, gpiod::line& line, int revs)
{
	double min { 10000.0 };
	double max { 0.0 };
    std::vector<int64_t> buf (pts_per_rev * revs);
    while(1)
    {
        // gather some data
    	auto dd = buf.begin();
    	while(dd != buf.end())
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
        if(stop.stop_requested())
		{
			std::cout << "max = " << max << ", min = " << min << "\n";
            return;
		}
		if(buf.empty())
            std::cout << "No signal (pulse)\r";
        else
        {        
            // process buf to rpms
            auto duration = buf.back() - buf.front();
            auto rpm = double(60000000000) * (buf.size() - 1) / pts_per_rev / double(duration);
			if(rpm > max)
				max = rpm;
			if(rpm < min)
				min = rpm;
            std::cout << rpm << " (pulse)\n";
        }
    }
}

void tacho_id(std::stop_token stop, gpiod::line& line, int revs)
{
    std::vector<int64_t> buf (revs);
    while(1)
    {
    	auto dd = buf.begin();
		bool alt {true};
    	while(dd != buf.end())
	    {
		    if(line.event_wait(2000ms)) // expecting once per rev, so at least 1.55s
		    {
			    auto e = line.event_read();
				if(alt)
				{
					*dd = e.timestamp.count();
			    	++dd;
				}
				alt = !alt;
		    }
		    else
			    break;
		}
		if(stop.stop_requested())
			return;
		if(buf.empty())
			std::cout << "No signal (id)\r";
		else
		{        
			// process buf to rpms
			auto duration = buf.back() - buf.front();
			auto rpm = double(60000000000) * (buf.size() - 1) / double(duration);
			std::cout << rpm << " (id)\n";
#if 0
			// starting getting two close id pulses per rev...
			std::adjacent_difference(buf.begin(), buf.end(), buf.begin());
			for(auto n : buf)
				std::cout << n << "\n";
#endif
		}
	}
}

void welcome()
{
    std::cout << "encoder disc tachometer 0.03\n";
    std::cout << "Copyright (c) 2022 paul@epicyclism.com\n";
}

int main()
{
    welcome();
	auto lines = initialise<gpiod::line_request::EVENT_RISING_EDGE>(line_q, line_id);
    if(lines.empty())
    {
        std::cout << "Failed to intiialise GPIO!\n";
        return 1;
    }
    std::cout << "Starting tachometer...\n";

	std::jthread t   ( tacho, std::ref(lines[0]), 5);
  	std::jthread tid ( tacho_id, std::ref(lines[1]), 5);
  
    std::cout << "\n\npress 'q' and enter to exit.\n\n";
	std::string ln;
	while (std::getline(std::cin, ln))
	{
		switch (ln[0])
		{
		case 'q':
		case 'Q':
			goto exit;
		default:
			break;
		}
	}
exit:
	std::cout << "Waiting for tachometer exit...\n";
	// request stops so they both stop together
	t.request_stop();
	tid.request_stop();
	t.join();
	tid.join();
    std::cout << "done\n\n";
}
