#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(spi_slave, LOG_LEVEL_DBG);

/* ------------------------------------------------------------------ */
/*  Devicetree reference                                               */
/* ------------------------------------------------------------------ */
#define SPI_SLAVE_NODE  DT_NODELABEL(spi1)
#define BUF_SIZE        32

/* ------------------------------------------------------------------ */
/*  Buffers  (must be in RAM)                                          */
/* ------------------------------------------------------------------ */
static uint8_t tx_buf[BUF_SIZE];
static uint8_t rx_buf[BUF_SIZE];

/* ------------------------------------------------------------------ */
/*  Semaphore for async callback -> main thread signalling             */
/* ------------------------------------------------------------------ */
static K_SEM_DEFINE(spis_sem, 0, 1);
static int spis_result;

/* ------------------------------------------------------------------ */
/*  Async callback (runs in ISR context)                               */
/* ------------------------------------------------------------------ */
static void spi_slave_cb(const struct device *dev,
                         int                  result,
                         void                *userdata)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(userdata);

    spis_result = result;
    k_sem_give(&spis_sem);
}

/* ------------------------------------------------------------------ */
/*  SPI slave configuration                                            */
/* ------------------------------------------------------------------ */
static const struct spi_config spis_cfg = {
    /*
     * SPI_OP_MODE_SLAVE  : operate as peripheral/slave
     * SPI_WORD_SET(8)    : 8-bit words
     * SPI_TRANSFER_MSB   : MSB first
     *
     * Add SPI_MODE_CPOL / SPI_MODE_CPHA if your master uses Mode 1/2/3
     * Mode 0 (default): CPOL=0, CPHA=0
     * Mode 3          : CPOL=1, CPHA=1  -> SPI_MODE_CPOL | SPI_MODE_CPHA
     */
    .operation = SPI_OP_MODE_SLAVE  |
                 SPI_WORD_SET(8)    |
                 SPI_TRANSFER_MSB,
    .frequency = 0,    /* ignored in slave mode */
    .slave     = 0,
};

/* ------------------------------------------------------------------ */
/*  Helpers                                                            */
/* ------------------------------------------------------------------ */
static void prepare_tx_buf(void)
{
    /* Pre-load a response for the master to read */
    for (int i = 0; i < BUF_SIZE; i++) {
        tx_buf[i] = (uint8_t)(0xA0 + i);
    }
}

static void process_rx_data(int len)
{
    LOG_INF("Received %d bytes from master", len);
    LOG_HEXDUMP_INF(rx_buf, len, "RX");

    /* Echo: copy received data into tx for next transaction */
    memcpy(tx_buf, rx_buf, len);
}

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */
int main(void)
{
    int ret;

    LOG_INF("SPI Slave example - nRF52840 (nCS v3.2.1)");

    /* Get SPI device from devicetree */
    const struct device *spis_dev = DEVICE_DT_GET(SPI_SLAVE_NODE);

    if (!device_is_ready(spis_dev)) {
        LOG_ERR("SPI slave device not ready");
        return -ENODEV;
    }

    LOG_INF("SPI slave device ready");

    while (1) {
        /* Prepare buffers for next transaction */
        prepare_tx_buf();
        memset(rx_buf, 0, sizeof(rx_buf));

        struct spi_buf tx_spi_buf    = { .buf = tx_buf, .len = BUF_SIZE };
        struct spi_buf rx_spi_buf    = { .buf = rx_buf, .len = BUF_SIZE };
        struct spi_buf_set tx_set    = { .buffers = &tx_spi_buf, .count = 1 };
        struct spi_buf_set rx_set    = { .buffers = &rx_spi_buf, .count = 1 };

        /*
         * spi_transceive_cb() arms the slave and returns immediately.
         * The callback fires when a master transaction completes.
         */
        ret = spi_transceive_cb(spis_dev,
                                &spis_cfg,
                                &tx_set,
                                &rx_set,
                                spi_slave_cb,
                                NULL);
        if (ret < 0) {
            LOG_ERR("spi_transceive_cb failed: %d", ret);
            k_sleep(K_MSEC(100));
            continue;
        }

        LOG_DBG("Waiting for master transaction...");

        /* Block until callback signals completion */
        k_sem_take(&spis_sem, K_FOREVER);

        if (spis_result < 0) {
            LOG_ERR("SPI transfer error: %d", spis_result);
            continue;
        }

        /* Process the received data */
        process_rx_data(BUF_SIZE);
    }

    return 0;
}