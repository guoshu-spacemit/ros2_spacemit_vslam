// Keep a module-qualified guard distinct from same-named headers in other SDKs.
// NOLINTNEXTLINE(build/header_guard)
#ifndef SPACEMIT_MAPPING_CORE_IMAGE_VIEW_H_
// NOLINTNEXTLINE(build/header_guard)
#define SPACEMIT_MAPPING_CORE_IMAGE_VIEW_H_

#include <cstddef>
#include <limits>
#include <stdexcept>

namespace spacemit_mapping
{

// Read-only borrowed image. Stride and accessible_count are measured in Pixel
// elements. The owner must keep the backing allocation alive for every access.
template <class Pixel> class ImageView
{
public:
    ImageView() = default;

    ImageView(const Pixel* data, std::size_t accessible_count, std::size_t width,
        std::size_t height, std::size_t row_stride = 0)
    {
        if (width == 0 || height == 0)
        {
            if (width != 0 || height != 0 || accessible_count != 0 || row_stride != 0)
                throw std::invalid_argument(
                    "Empty images require zero dimensions, count and stride");
            return;
        }
        if (data == nullptr)
            throw std::invalid_argument("Nonempty image requires data");
        const std::size_t stride = row_stride == 0 ? width : row_stride;
        if (stride < width)
            throw std::invalid_argument("Image stride is smaller than width");
        if (height - 1 > (std::numeric_limits<std::size_t>::max() - width) / stride)
            throw std::overflow_error("Image element extent overflows size_t");
        const std::size_t required_count = (height - 1) * stride + width;
        if (required_count > accessible_count)
            throw std::invalid_argument("Image backing allocation is too small");
        data_ = data;
        width_ = width;
        height_ = height;
        row_stride_ = stride;
    }

    const Pixel* data() const noexcept { return data_; }
    std::size_t width() const noexcept { return width_; }
    std::size_t height() const noexcept { return height_; }
    std::size_t row_stride() const noexcept { return row_stride_; }
    bool empty() const noexcept { return width_ == 0; }

    const Pixel* row(std::size_t y) const
    {
        if (y >= height_)
            throw std::out_of_range("Image row is outside the view");
        return data_ + y * row_stride_;
    }

    const Pixel& at(std::size_t x, std::size_t y) const
    {
        if (x >= width_)
            throw std::out_of_range("Image column is outside the view");
        return row(y)[x];
    }

private:
    const Pixel* data_ = nullptr;
    std::size_t width_ = 0;
    std::size_t height_ = 0;
    std::size_t row_stride_ = 0;
};

} // namespace spacemit_mapping

#endif // SPACEMIT_MAPPING_CORE_IMAGE_VIEW_H_
