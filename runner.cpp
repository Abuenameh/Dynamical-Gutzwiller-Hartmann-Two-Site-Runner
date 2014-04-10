/* 
 * File:   runner.cpp
 * Author: Abuenameh
 *
 * Created on 12 November 2013, 21:59
 */

#include <cstdlib>
#include <sstream>
#include <string>

using namespace std;

#include <boost/range/irange.hpp>
#include <boost/assign.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/progress.hpp>
#include <boost/process.hpp>
#include <boost/iostreams/stream.hpp>

template<typename T, typename InputIterator>
void Print(std::ostream& ostr,
        InputIterator itbegin,
        InputIterator itend,
        const std::string& delimiter) {
    std::copy(itbegin,
            itend,
            std::ostream_iterator<T>(ostr, delimiter.c_str()));
}

struct Input {
    int seed;
    double Wi;
    double Wf;
    double mu;
    double D;
    double dt;
    int dnsav;
};

struct Point {
    int resi;
    int tau;
};

boost::mutex pointsmutex;

boost::mutex progressmutex;

void taupoints(int i, Input& in, queue<Point>& points, boost::progress_display& progress) {

//    ostringstream outputoss;
//    outputoss << "/Users/Abuenameh/NetBeansProjects/Dynamical Gutzwiller Lin Canonical Runner Threads/dist/Release/GNU-MacOSX/thread";
//    outputoss << i;
//    outputoss << ".txt";
////    boost::filesystem::path p = outputoss.str();
//    boost::iostreams::file_descriptor_sink output(outputoss.str());
////    string output = "/Users/Abuenameh/NetBeansProjects/Dynamical Gutzwiller Lin Canonical Runner Threads/dist/Release/GNU-MacOSX/thread" + i + ".txt";
    
    for (;;) {
        Point point;
        {
            boost::mutex::scoped_lock lock(pointsmutex);
            if (points.empty()) {
                break;
            }
            point = points.front();
            points.pop();
        }

        //	cout << "Thread " << i << endl;

#ifdef AMAZON
        string prog = "/home/ubuntu/dynamical_gutzwiller_hartmann_two_site";
#elif defined(FSTSERVER)
        string prog = "C:/Users/Abuenameh/Documents/NetBeansProjects/Dynamical Gutzwiller Hartmann Two Site/dist/Release/MinGW_TDM-Windows/dynamical_gutzwiller_hartmann_two_site.exe";
#else
        string prog = "/Users/Abuenameh/NetBeansProjects/Dynamical Gutzwiller Hartmann Two Site/dist/Release/CLang-MacOSX/dynamical_gutzwiller_hartmann_two_site";
#endif
        vector<string> args;
        args.push_back(prog);
        args.push_back(boost::lexical_cast<string>(in.seed));
        args.push_back(boost::lexical_cast<string>(in.Wi));
        args.push_back(boost::lexical_cast<string>(in.Wf));
        args.push_back(boost::lexical_cast<string>(in.mu));
        args.push_back(boost::lexical_cast<string>(in.D));
        args.push_back(boost::lexical_cast<string>(point.tau) + "e-9");
        args.push_back(boost::lexical_cast<string>(in.dt));
        args.push_back(boost::lexical_cast<string>(in.dnsav));
        args.push_back(boost::lexical_cast<string>(point.resi));
//        	Print<string>(cout, args.begin(), args.end(), ",");
        
        boost::process::child child = boost::process::execute(boost::process::initializers::set_args(args), boost::process::initializers::close_stdout(), boost::process::initializers::inherit_env());
        boost::process::wait_for_exit(child);

//        boost::process::context ctx;
//        ctx.stdout_behavior = boost::process::inherit_stream();
//        ctx.stderr_behavior = boost::process::inherit_stream();
//        boost::process::child child = boost::process::launch(prog, args, ctx);
//        boost::process::status stat = child.wait();

        {
            boost::mutex::scoped_lock lock(progressmutex);
            ++progress;
        }

    }

}

/*
 *
 */
int main(int argc, char** argv) {

    int seed = boost::lexical_cast<int>(argv[1]);
    double Wi = boost::lexical_cast<double>(argv[2]);
    double Wf = boost::lexical_cast<double>(argv[3]);
    double mu = boost::lexical_cast<double>(argv[4]);
    double D = boost::lexical_cast<double>(argv[5]);
    double dt = boost::lexical_cast<double>(argv[6]);
    int dnsav = boost::lexical_cast<int>(argv[7]);
    int res0 = boost::lexical_cast<int>(argv[8]);
    int tau0 = boost::lexical_cast<int>(argv[9]);
    int tauf = boost::lexical_cast<int>(argv[10]);
    int dtau = boost::lexical_cast<int>(argv[11]);
    int numthreads = boost::lexical_cast<int>(argv[12]);

    vector<int> taus = boost::assign::list_of(tau0).range(boost::irange(tau0 + dtau, tauf + dtau, dtau));

    Input in;
    in.seed = seed;
    in.Wi = Wi;
    in.Wf = Wf;
    in.mu = mu;
    in.D = D;
    in.dt = dt;
    in.dnsav = dnsav;

    boost::progress_display progress(taus.size());
    queue<Point> points;

    BOOST_FOREACH(int tau, taus) {
        Point point;
        point.resi = res0++;
        point.tau = tau;
        points.push(point);
    }


    boost::thread_group threads;
    for (int i = 0; i < numthreads; i++) {
        threads.create_thread(boost::bind(&taupoints, i, boost::ref(in), boost::ref(points), boost::ref(progress)));
    }
    threads.join_all();

    return 0;
}

