// AliOS Things
#include <aos/kernel.h>
#include <aos/yloop.h>

// AOS event tags and callbacks
namespace aos {

static constexpr auto bt_tag = 0x0504;
enum class BtEvent : int
	{ Start
	, Stop
	, Connect
	, Disconnect
};

void bt_event_received(input_event_t* event, void* private_data);

} // namespace aos_callbacks
