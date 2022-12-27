#include "sdk_common.h"

#define NRF_LOG_MODULE_NAME bond_mgr
#if BOND_MGR_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       BOND_MGR_CONFIG_LOG_LEVEL
#define NRF_LOG_INFO_COLOR  BOND_MGR_CONFIG_INFO_COLOR
#define NRF_LOG_DEBUG_COLOR BOND_MGR_CONFIG_DEBUG_COLOR
#else // BOND_MGR_CONFIG_LOG_ENABLED
#define NRF_LOG_LEVEL       0
#endif // BOND_MGR_CONFIG_LOG_ENABLED
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();


#include "nrf_log_ctrl.h"

#include "bond_mgr.h"

#define MAX_NUM_BONDED_DEVICES_STORED 5
#define MAX_TOTAL_BONDED_DEVICES_STORED MAX_NUM_BONDED_DEVICES_STORED

static pm_peer_id_t peer_deletion_table[MAX_NUM_BONDED_DEVICES_STORED] = { PM_PEER_ID_INVALID, PM_PEER_ID_INVALID, PM_PEER_ID_INVALID, PM_PEER_ID_INVALID, PM_PEER_ID_INVALID };
static uint8_t num_peers_to_delete = 0;

void bond_mgr_init(pm_evt_handler_t pm_evt_handler)
{
    // Register a handler for peer events
    ret_code_t err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static ret_code_t add_peer_to_deletion_table(pm_peer_id_t peer_id)
{
    if (peer_id == PM_PEER_ID_INVALID)
    {
        NRF_LOG_WARNING("Peer invalid");
        return NRF_ERROR_INVALID_DATA;
    }
  
    for (uint8_t i = 0; i < MAX_TOTAL_BONDED_DEVICES_STORED; i++)
    {
        // Peer ID is already in the table
        if (peer_deletion_table[i] == peer_id)
        {
            NRF_LOG_WARNING("Peer already saved for deletion");
            return NRF_ERROR_INVALID_PARAM;
        }
  
        // Found the first empty slot in the table, use it
        if (peer_deletion_table[i] == PM_PEER_ID_INVALID)
        {
            NRF_LOG_INFO("Peer successfully saved for deletion");
            peer_deletion_table[i] = peer_id;
  
        //Keep track of how many peers are queued for deletion
            num_peers_to_delete++;
            return NRF_SUCCESS;
        }
    }
  
    // Not enough space in the peer deletion table
    NRF_LOG_WARNING("Peer deletion table is full!");
    return NRF_ERROR_NO_MEM;
}
static void remove_peer_from_deletion_table(pm_peer_id_t peer_id)
{
    NRF_LOG_INFO("Num peers to delete: %d", num_peers_to_delete);
    // Clear peer ID from table
    for (uint8_t i = 0; i < MAX_TOTAL_BONDED_DEVICES_STORED; i++)
    {
        if (peer_deletion_table[i] == peer_id)
        {
            peer_deletion_table[i] = PM_PEER_ID_INVALID;
            //Keep track of how many peers remain in the table
            num_peers_to_delete--;
            //Delete the next queued peer (if any)
            bond_mgr_delete_queued_peers();
        }
    }
}

bool bond_mgr_deletion_pending()
{
    return num_peers_to_delete > 0;
}

ret_code_t bond_mgr_queue_peer_for_deletion(pm_peer_id_t peer_id_to_del)
{
    return add_peer_to_deletion_table(peer_id_to_del);
}

void bond_mgr_delete_queued_peers()
{
    if (num_peers_to_delete == 0)
    {
        // TODO: This is where any interested module should
        // be notified that peer deletion is complete!
        return;
    }
  
    for (uint8_t i = 0; i < MAX_TOTAL_BONDED_DEVICES_STORED; i++)
    {
        if (peer_deletion_table[i] != PM_PEER_ID_INVALID)
        {
            ret_code_t err_code = pm_peer_delete(peer_deletion_table[i]);
            if (err_code != NRF_SUCCESS)
            {
                // In this design, if deletion fails, we give up and                      
                // remove it from the deletion table. Depending on
                // your application, you may want to handle this failure
                // differently.
                remove_peer_from_deletion_table(peer_deletion_table[i]);
            }
            else
            {
                NRF_LOG_INFO("Waiting for peer deletion to complete");
                // Peer deletion occurs asynchronously, so a peer
          // delete event (PM_PEER_DELETE_SUCCEEDED or 
          // PM_PEER_DELETE_FAILED) will indicate success or 
          // failure of the deletion
                return;
            }
        }
    }
  
}

