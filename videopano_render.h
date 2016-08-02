#include <cstddef>
#include "videopano_typedef.h"

namespace adobe_agt {
namespace videopano {

void videopano_render_image_cylindrical(unsigned char* dst, std::ptrdiff_t dst_row_bytes,
                                        std::size_t dst_width, std::size_t dst_height,
                                        float fl, float ic1, float ic2,
                                        const unsigned char* src, std::ptrdiff_t src_row_bytes,
                                        std::size_t src_width, std::size_t src_height,
                                        const Image_Intrinsic& intrinsic,
                                        const Matrix33& rot);

} // namespace videopano
} // namespace adobe_agt
