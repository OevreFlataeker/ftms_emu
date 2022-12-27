#ifndef BOND_MANAGER_H__
#define BOND_MANAGER_H__

#include "sdk_errors.h"
#include "peer_manager.h"

void        bond_mgr_init();
void        bond_mgr_delete_queued_peers();
ret_code_t  bond_mgr_queue_peer_for_deletion(pm_peer_id_t peer_id_to_del);
bool        bond_mgr_deletion_pending();
//ret_code_t  peer_mgr_register_peer_delete_handler(
//            peer_delete_handler handler);

#endif