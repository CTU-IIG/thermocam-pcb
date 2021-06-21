#ifndef CUSTOM_ACCUMULATORS_HPP
#define CUSTOM_ACCUMULATORS_HPP

#include <boost/accumulators/statistics/rolling_variance.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_moment.hpp>

// Extend boost::acumulators
namespace boost { namespace accumulators {
namespace impl {
template<typename Sample>
struct rolling_stddev_impl
    : accumulator_base
{
    // for boost::result_of
    typedef Sample result_type;

    rolling_stddev_impl(dont_care) {}

    template<typename Args>
    result_type result(Args const &args) const
    {
        result_type var = rolling_variance(args);
        return sqrt(var);
    }

    // serialization is done by accumulators it depends on
    template<class Archive>
    void serialize(Archive & ar, const unsigned int file_version) {}
};
}
namespace tag {
// lazy_rolling_variance that uses lazy_rolling_mean (not immediate_rolling_mean)
struct really_lazy_rolling_variance
    : depends_on<
        rolling_count,
        lazy_rolling_mean, // this is the difference from the original boost implementation
        rolling_moment<2> >
{
    typedef impl::lazy_rolling_variance_impl< boost::mpl::_1 > impl;
};
}
// for the purposes of feature-based dependency resolution,
// really_lazy_rolling_variance provides the same feature as rolling_variance
template<>
struct feature_of<tag::really_lazy_rolling_variance>
    : feature_of<boost::accumulators::tag::rolling_variance>
{
};
namespace tag {
struct rolling_stddev
    : depends_on<really_lazy_rolling_variance>
{
    typedef boost::accumulators::impl::rolling_stddev_impl< boost::mpl::_1 > impl;
};
}
namespace extract
{
    extractor<tag::rolling_stddev> const rolling_stddev = {};
    BOOST_ACCUMULATORS_IGNORE_GLOBAL(rolling_stddev)
}
using extract::rolling_stddev;
}}

#endif // CUSTOM_ACCUMULATORS_HPP
