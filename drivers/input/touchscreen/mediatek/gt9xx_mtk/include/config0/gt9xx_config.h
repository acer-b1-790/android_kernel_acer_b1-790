#ifndef _GT9XX_CONFIG_H_
#define _GT9XX_CONFIG_H_

/* ***************************PART2:TODO define********************************** */
/* STEP_1(REQUIRED):Change config table. */
/*TODO: puts the config info corresponded to your TP here, the following is just
a sample config, send this config should cause the chip cannot work normally*/
#define CTP_CFG_GROUP0 {\
0x48,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x08,0x23,0x08,0x50,0x37,0x03,0x06,0x00,0x00,0x00,0x00,0x11,0x10,0x00,0x15,0x17,0x1C,0x14,0x89,0x09,0x0A,0x3C,0x00,0xD3,0x07,0x00,0x00,0x00,0x83,0x02,0x1D,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,0x98,0x2C,0x00,0x8A,0x34,0x00,0x7C,0x3F,0x00,0x72,0x4C,0x00,0x69,0x5B,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x05,0x04,0x02,0x00,0x08,0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x01 \
}

#define GTP_CFG_GROUP0_CHARGER {\
}

/* TODO puts your group2 config info here,if need. */
#define CTP_CFG_GROUP1 {\
0x48,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x08,0x23,0x08,0x50,0x37,0x03,0x06,0x00,0x00,0x00,0x00,0x11,0x10,0x00,0x15,0x17,0x1C,0x14,0x89,0x09,0x0A,0x3C,0x00,0xD3,0x07,0x00,0x00,0x00,0x83,0x02,0x1D,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,0x98,0x2C,0x00,0x8A,0x34,0x00,0x7C,0x3F,0x00,0x72,0x4C,0x00,0x69,0x5B,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x05,0x04,0x02,0x00,0x08,0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x01 \
}

/* TODO puts your group2 config info here,if need. */
#define GTP_CFG_GROUP1_CHARGER {\
}

/* TODO puts your group3 config info here,if need. */
#define CTP_CFG_GROUP2 {\
0x48,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x08,0x23,0x08,0x50,0x37,0x03,0x06,0x00,0x00,0x00,0x00,0x11,0x10,0x00,0x15,0x17,0x1C,0x14,0x89,0x09,0x0A,0x3C,0x00,0xD3,0x07,0x00,0x00,0x00,0x83,0x02,0x1D,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,0x98,0x2C,0x00,0x8A,0x34,0x00,0x7C,0x3F,0x00,0x72,0x4C,0x00,0x69,0x5B,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x05,0x04,0x02,0x00,0x08,0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x01 \
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP2_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 3 here, if needed */
#define CTP_CFG_GROUP3 {\
0x48,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x08,0x23,0x08,0x50,0x37,0x03,0x06,0x00,0x00,0x00,0x00,0x11,0x10,0x00,0x15,0x17,0x1C,0x14,0x89,0x09,0x0A,0x3C,0x00,0xD3,0x07,0x00,0x00,0x00,0x83,0x02,0x1D,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,0x98,0x2C,0x00,0x8A,0x34,0x00,0x7C,0x3F,0x00,0x72,0x4C,0x00,0x69,0x5B,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x05,0x04,0x02,0x00,0x08,0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x01 \
}

/* TODO puts your group3 config info here,if need. */
#define GTP_CFG_GROUP3_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 4 here, if needed */
#define CTP_CFG_GROUP4 {\
0x48,0xD0,0x02,0x00,0x05,0x05,0x34,0x00,0x01,0x08,0x23,0x08,0x50,0x37,0x03,0x06,0x00,0x00,0x00,0x00,0x11,0x10,0x00,0x15,0x17,0x1C,0x14,0x89,0x09,0x0A,0x3C,0x00,0xD3,0x07,0x00,0x00,0x00,0x83,0x02,0x1D,0x00,0x01,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x64,0x94,0xC5,0x02,0x07,0x00,0x00,0x04,0x98,0x2C,0x00,0x8A,0x34,0x00,0x7C,0x3F,0x00,0x72,0x4C,0x00,0x69,0x5B,0x00,0x69,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x03,0x00,0x08,0x0A,0x0C,0x0E,0x10,0x12,0x14,0x16,0x18,0x1A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x05,0x04,0x02,0x00,0x08,0x0A,0x0C,0x0E,0x1D,0x1E,0x1F,0x20,0x22,0x24,0x28,0x29,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xCC,0x01 \
}

/* TODO puts your group4 config info here,if need. */
#define GTP_CFG_GROUP4_CHARGER {\
}

/* TODO: define your config for Sensor_ID == 5 here, if needed */
#define CTP_CFG_GROUP5 {\
}

/* TODO puts your group5 config info here,if need. */
#define GTP_CFG_GROUP5_CHARGER {\
}


#endif /* _GT1X_CONFIG_H_ */
