#include "regdma_rearm.h"

#include "esp_private/esp_pau.h"
#include "hal/pau_ll.h"
#include "soc/soc_caps.h"

esp_err_t c5vrx2_regdma_arm(uint32_t lp_link_root)
{
#if SOC_PAU_SUPPORTED
    if (lp_link_root == 0u) return ESP_ERR_INVALID_ARG;
    /* ESP32-C5 uses one always-on REGDMA entry address. Keep the four modem
     * write nodes in LP SRAM so the chain remains fetchable while HP SRAM is
     * lent to the MAC dump engine. */
    pau_regdma_link_addr_t entries = {0};
    entries[0] = (void *)(uintptr_t)lp_link_root;
    pau_regdma_set_entry_link_addr(&entries);
    pau_ll_clear_regdma_backup_done_intr_state(&PAU);
    pau_ll_clear_regdma_backup_error_intr_state(&PAU);
    return ESP_OK;
#else
    (void)lp_link_root;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}
