#include <mavlink/crater/crater.hpp>
#include <mavlink/message.hpp>

namespace mavlink {
/**
* Return message entry data for msgid.
*
* @note user of MAVLink library should provide
*       implementation for this function.
*       Use mavlink::<dialect-name>::MESSAGE_ENTRIES array to make hash map.
*
* @returns nullptr  if message is unknown
*/
const mavlink_msg_entry_t* mavlink_get_msg_entry(uint32_t msgid) {
    uint32_t low = 0, high = crater::MESSAGE_ENTRIES.size() - 1;
    while (low < high) {
        uint32_t mid = (low + 1 + high) / 2;
        if (msgid < crater::MESSAGE_ENTRIES[mid].msgid) {
            high = mid - 1;
            continue;
        }
        if (msgid > crater::MESSAGE_ENTRIES[mid].msgid) {
            low = mid;
            continue;
        }
        low = mid;
        break;
    }
    if (crater::MESSAGE_ENTRIES[low].msgid != msgid) {
        // msgid is not in the table
        return NULL;
    }
    return &crater::MESSAGE_ENTRIES[low];
}

} // namespace mavlink