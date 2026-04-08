// dblink: thin wrapper — wires up db_reader (RX) + db_sender (TX queue)

#include "dblink.h"
#include "db_reader.h"
#include "db_sender.h"

void dblink_setup(void) {
    db_reader_setup();
    db_sender_setup();
}
