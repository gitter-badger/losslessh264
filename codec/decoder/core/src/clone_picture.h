#ifndef __CLONE_PICTURE_H_
#define __CLONE_PICTURE_H_

#include <vector>

#include "picture.h"

namespace WelsDec {
	// a picture with automatically managed storage & some analytic accessors
	class LumaPicture {
	public:
		inline LumaPicture() : m_width(0), m_height(0), m_mb_size(0), m_b_size(0) { }
		LumaPicture(const SPicture &pic);
		// XXX analysis should be in a separate class

		// get the unnormalized (e.g., sum of all values) at specified macroblock
		int get_sum(int iMbX, int iMbY) const;
		std::vector<uint16_t> get_b_sum(int iMbX, int iMbY) const;

		uint8_t& at(int x, int y);
		const uint8_t& at(int x, int y) const;
		void set_mb_size(int newSize);

		inline int width() const { return m_width; }
		inline int height() const { return m_height; }

	private:
		// XXX generalize to support other data 
		std::vector<uint8_t> m_data;
		int m_width;
		int m_height;
		int m_mb_size;
		int m_b_size;
	};

	extern LumaPicture *gLastPicture;
}

#endif  // __CLONE_PICTURE_H_