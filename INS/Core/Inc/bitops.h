#ifndef BITOPS_H
#define BITOPS_H

#define BIT_SET(reg, b)\
	((reg) |= (1U<<(b)))

#define BIT_CLEAR(reg, b)\
	((reg) &= ~(1U<<(b)))

#define BIT_CHECK(reg, b)\
	(((reg)>>(b)) & 1U)

#define BIT_TOGGLE(reg, b)\
	((reg) ^= (1U<<(b)))

#define BITMASK_SET(reg, mask)\
	((reg) |= (mask))

#define BITMASK_GET(reg, mask)\
	((reg) & (mask))

#define BITMASK_CLEAR(reg, mask)\
	((reg) &= (~(mask)))

#define BITMASK_CLEAR_AND_SET(reg, cmask, smask)\
	((reg) = (((reg) & (~(cmask))) | (smask)))

#define BITMASK_CHECK(reg, mask)\
	(((reg) & (mask)) == (mask))


#endif //BITOPS_H
