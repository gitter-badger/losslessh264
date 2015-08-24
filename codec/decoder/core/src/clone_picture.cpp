#include <assert.h>

#include "clone_picture.h"

const int MB_SIZE = 16;
const int B_SIZE = 4;

namespace WelsDec {

LumaPicture::LumaPicture(const SPicture &pic) :
	m_width(pic.iWidthInPixel), m_height(pic.iHeightInPixel), m_mb_size(MB_SIZE), m_b_size(B_SIZE) {
	m_data.resize(m_width * m_height);
	for (int y = 0; y < m_height; ++y) {
		for (int x = 0; x < m_width; ++x) {
			at(x, y) = *(pic.pBuffer[0] + y * pic.iLinesize[0] + x);
		}
	}
}

uint8_t& LumaPicture::at(int x, int y) {
	return m_data[m_width * y + x];
}

const uint8_t& LumaPicture::at(int x, int y) const {
	return const_cast<LumaPicture*>(this)->at(x, y);
}

void LumaPicture::set_mb_size(int newSize) {
	m_mb_size = newSize;
}

// full macroblock average
int LumaPicture::get_sum(int iMbX, int iMbY) const {
	int sum = 0;
	for (int dy = 0; dy < m_mb_size; ++dy) {
		for (int dx = 0; dx < m_mb_size; ++dx) {
			int x = iMbX * m_mb_size + dx, y = iMbY * m_mb_size + dy;
			int value = at(x, y);
			// fprintf(stderr, "%d %d => %d %d [%d]\n", iMbX, iMbY, x, y, value);
			sum += value;
		}
	}
	return sum;
}

// block average, raster order
std::vector<uint16_t> LumaPicture::get_b_sum(int iMbX, int iMbY) const {
	std::vector<uint16_t> result;
	// block coordinate
	for (int by = 0; by < m_mb_size / m_b_size; ++by) {
		for (int bx = 0; bx < m_mb_size / m_b_size; ++bx) {
			// pixel within block
			int sum = 0;
			for (int dy = 0; dy < m_b_size; ++dy) {
				for (int dx = 0; dx < m_b_size; ++dx) {
					int x = iMbX * m_mb_size + bx * m_b_size + dx,
					    y = iMbY * m_mb_size + by * m_b_size + dy;
					int value = at(x, y);
					// fprintf(stderr, "%d %d => %d %d [%d]\n", iMbX, iMbY, x, y, value);
					sum += value;
				}
			}
			assert(sum < (1 << (sizeof(uint16_t))));
			result.push_back(sum);
		}
	}
	return result;
}

// XXX get rid of static initialization -- use __once?
LumaPicture *gLastPicture;

}
