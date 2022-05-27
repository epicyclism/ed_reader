//
// calibrate.cpp
//
// attempt to characterise the out of round of the encoder disc
//
// Copyright (c) 2004-2022 Paul Ranson. paul@epicyclism.com
//
// Refer to licence in repository.
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
#include <tuple>
#include <utility>

#include "gpio_help.h"
#include "basic_file.h"

using namespace std::literals;

// 10 revs + 
constexpr size_t samples { 5500 };
constexpr size_t ticks { 500 };

auto get_data(auto& lines)
{
    std::vector<int64_t> vi;
    std::vector<int64_t> vq;
    std::vector<int64_t> vid;
    vi.reserve(samples);
    vq.reserve(samples);
    vid.reserve(samples / ticks);

    while((vi.size() < samples) && (vq.size() < samples))
    {
        auto lb = lines.event_wait(1000ms);
        if( lb.empty())
        {
            std::cerr << "event_wait timeout\n";
            break;
        }
        for(auto& l : lb)
        {
            auto le = l.event_read();
            switch(le.source.offset())
            {
                case line_i:
                    vi.push_back(le.timestamp.count());
                    break;
                case line_q:
                    vq.push_back(le.timestamp.count());
                    break;
                case line_id:
                    vid.push_back(le.timestamp.count());
                    break;
                default:
                    std::cerr << "Unknown line event source " << le.source.offset() << "\n";
                    break;
            }
        }
    }

    return std::make_tuple(vi, vq, vid);
}

void welcome()
{
    std::cerr << "calib_aq 0.02\n";
    std::cerr << "Copyright (c) 2022 paul@epicyclism.com\n\n";
    std::cerr << "Capturing 10 revolutions...";
}

void process_and_report(std::vector<int64_t>& v, int64_t st, char id)
{
    // report rpm
    auto duration = v.back() - v.front();
    auto rpm = double(60000000000) * (v.size() - 1) / pts_per_rev / double(duration);
    std::cout << "# rpm for this phase " << rpm << "\n";
    std::cout << "# id " << id << " (note values are nanoseconds per 1/500th of a rev)\n";
    // find the start
    auto ip = std::upper_bound(v.begin(), v.end(), st);
    // remove the lead in
    v.erase(v.begin(), ip);
    // turn to intervals
	std::adjacent_difference(v.begin(), v.end(), v.begin());
	v.erase(v.begin());
    // calculate means
    std::vector<int64_t> m (pts_per_rev);
    std::vector<int>     mc(pts_per_rev);
    auto im { m.begin()};
    auto ic { mc.begin()};
    for(auto interval : v)
    {
        *im++ += interval;
        *ic++ += 1;
        if(im == m.end())
        {
            im = m.begin();
            ic = mc.begin();
        }
    }
    std::transform(m.begin(), m.end(), mc.begin(), m.begin(), std::divides<>());
    // output
	double angle { 0.0 };
	double angle_inc {std::numbers::pi * 2.0 / pts_per_rev};
	std::for_each(m.begin(), m.end(), [&angle, angle_inc](auto vel){ std::cout << angle << " " << vel << "\n"; angle += angle_inc;});
    std::cout << "\n";
}

int main()
{
    welcome();
	auto lines = initialise<gpiod::line_request::EVENT_RISING_EDGE>(line_i, line_q, line_id);
    auto[vi, vq, vid] = get_data(lines);
    if( vid.empty())
    {
        std::cout << "# no id mark found.\n\n";
        return 1;
    }
    process_and_report(vi, vid.front(), 'i');
    process_and_report(vq, vid.front(), 'q');
}
