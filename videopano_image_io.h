#include <boost/gil/typedefs.hpp>

namespace adobe_agt {
namespace videopano {

void videopano_read_image (const char* filename, boost::gil::rgb8_image_t& im);
void videopano_write_image(const char* filename, const boost::gil::rgb8_image_t& im);

} // namespace videopano
} // namespace adobe_agt
