
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    using namespace msr::airlib;

    struct NamedTest
    {
        const char* name;
        std::unique_ptr<TestBase> test;
    };

    NamedTest tests[] = {
        { "Quaternion", std::unique_ptr<TestBase>(new QuaternionTest()) },
        { "Celestial", std::unique_ptr<TestBase>(new CelestialTest()) },
        { "Settings", std::unique_ptr<TestBase>(new SettingsTest()) },
        { "SimpleFlight", std::unique_ptr<TestBase>(new SimpleFlightTest()) }
        //,
        //std::unique_ptr<TestBase>(new PixhawkTest()),
        //std::unique_ptr<TestBase>(new WorkerThreadTest())
    };

    bool matched = argc == 1;
    for (auto& named_test : tests) {
        bool selected = argc == 1;
        for (int argi = 1; argi < argc && !selected; ++argi)
            selected = std::string(argv[argi]) == named_test.name;

        if (selected) {
            matched = true;
            named_test.test->run();
        }
    }

    if (!matched) {
        std::cerr << "Unknown test name. Available: Quaternion Celestial Settings SimpleFlight\n";
        return 2;
    }

    return 0;
}
