/*
 * RFNoC_Utils.h
 *
 *  Created on: Dec 28, 2016
 *      Author: Patrick
 */

#ifndef RFNOC_UTILS_H_
#define RFNOC_UTILS_H_

#include <uhd/device3.hpp>

/*
 * A structure representing the block ID and port
 */
struct BlockInfo {
    uhd::rfnoc::block_id_t blockID;
    size_t port;
};

bool operator==(const BlockInfo &lhs, const BlockInfo &rhs);

BlockInfo findAvailableSink(const uhd::device3::sptr usrp, const std::string &blockID);

BlockInfo findAvailableSource(const uhd::device3::sptr usrp, const std::string &blockID);

#endif /* RFNOC_UTILS_H_ */
