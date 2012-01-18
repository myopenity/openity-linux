/*
 * Copyright (C) 2011 Ilya Yanok, Emcraft Systems
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TI_DAVINCI_EMAC
void am35xx_ethernet_init(unsigned long mdio_bus_freq, int rmii_en);
#else
static inline void am35xx_ethernet_init(unsigned long mdio_bus_freq,
		int rmii_en)
{
}
#endif
