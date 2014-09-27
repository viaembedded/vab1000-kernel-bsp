/*
 * I2C Controller Driver for S3 Elite SoC Chips
 */
#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_i2c.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <asm/irq.h>


#define ELITE_I2C_REG(x)   (x)

#define CTRL_REG       ELITE_I2C_REG(0x00)
#define XFER_REG       ELITE_I2C_REG(0x02)
#define STAT_REG       ELITE_I2C_REG(0x04)
#define INT_STAT_REG   ELITE_I2C_REG(0x06)
#define INT_MARK_REG   ELITE_I2C_REG(0x08)
#define DATAIO_REG     ELITE_I2C_REG(0x0A)
#define TIME_REG       ELITE_I2C_REG(0x0C)
#define CLK_DIV_REG    ELITE_I2C_REG(0x0E)
/* slave I2C registers */
#define S_CTRL_REG     ELITE_I2C_REG(0x10)
#define S_STAT_REG     ELITE_I2C_REG(0x12)
#define S_INT_STAT_REG ELITE_I2C_REG(0x14)
#define S_INT_MARK_REG ELITE_I2C_REG(0x16)
#define S_DATAIO_REG   ELITE_I2C_REG(0x18)
#define S_TIME_REG     ELITE_I2C_REG(0x1A)

/*
 * I2C Module Status Info
 */
#define I2C_READY                        0x0002
#define I2C_BUSY                         0x0000
#define I2C_STATUS_MASK                  0x0002
#define I2C_CSR_RCV_ACK                  0x0000
#define I2C_CSR_RCV_NOT_ACK              0x0001
#define I2C_CSR_RCV_ACK_MASK             0x0001

/*
 * I2C Controller Control Info
 */
#define I2C_CR_CPU_RDY                   0x0008
#define I2C_CR_TX_END                    0x0004
#define I2C_CR_TX_NEXT_NO_ACK            0x0002
#define I2C_CR_TX_NEXT_ACK               0x0000
#define I2C_CR_ENABLE                    0x0001
#define I2C_SLAV_MODE_SEL                0x8000


/*
 * I2C Transfer Control Info
 */
#define I2C_TCR_HS_MODE                 0x2000
#define I2C_TCR_STANDARD_MODE           0x0000
#define I2C_TCR_FAST_MODE               0x8000
#define I2C_TCR_MASTER_WRITE            0x0000
#define I2C_TCR_MASTER_READ             0x4000
#define I2C_TCR_SLAVE_ADDR_MASK         0x007F

/*
 * I2C Interrupt Status Info
 */
#define I2C_ISR_SCL_TIME_OUT             0x0004
#define I2C_ISR_SCL_TIME_OUT_WRITE_CLEAR 0x0004
#define I2C_ISR_BYTE_END                 0x0002
#define I2C_ISR_BYTE_END_WRITE_CLEAR     0x0002
#define I2C_ISR_NACK_ADDR                0x0001
#define I2C_ISR_NACK_ADDR_WRITE_CLEAR    0x0001
#define I2C_ISR_ALL_WRITE_CLEAR          0x0007

#define I2C_IMR_SCL_TIME_OUT_MASK        0x0004
#define I2C_IMR_BYTE_END_MASK            0x0002
#define I2C_IMR_NACK_ADDR_MASK           0x0001
#define I2C_IMR_ALL_ENABLE               0x0007

/*
 * I2C Data IO Info
 */
#define I2C_CDR_DATA_READ_MASK           0xFF00
#define I2C_CDR_DATA_WRITE_MASK          0x00FF

/*
 * I2C Timer Info
 */
#define I2C_TR_SCL_TIME_OUT_MASK         0xFF00
#define I2C_TR_FSTP_MASK                 0x00FF
#define I2C_TR_STD_VALUE                 0x8064
#define I2C_TR_FAST_VALUE                0x8019

/*
 * I2C Clock Divider Info
 */
#define APB_96M_I2C_DIV                  7
#define APB_166M_I2C_DIV                 12

#define MAX_RX_TIMEOUT                   500
#define MAX_MESSAGES                     65536

struct elite_isr_status {
    int      isr_nack;
    int      isr_byte_end;
    int      isr_timeout;
    int      isr_int_pending;
};

enum elite_i2c_mode {
    I2C_STANDARD_MODE = 0,
    I2C_FAST_MODE     = 1,
    I2C_HS_MODE       = 2,
};

/* every adapter generates one of this struct */
struct elite_i2c_priv {
    int                    dev_id;
    unsigned int           master;

	struct clk             *clk;
    struct elite_isr_status    isr_stat;
    enum elite_i2c_mode        i2c_mode;

    spinlock_t             lock;
    wait_queue_head_t      wait;

    unsigned int           irq;
    void __iomem           *regs;
    struct resource        *ioarea;
    struct device          *dev;
    struct i2c_adapter     adap;
    bool is_suspended;//prevents i2c controller accesses after suspend is called

    /* adapter transfer opertions */
    int (*write_msg)(struct elite_i2c_priv *priv, unsigned int slave_addr, char *buff, 
                                            unsigned int length, int restart, int last);
    int (*read_msg)(struct elite_i2c_priv *priv, unsigned int slave_addr, char *buff, 
                                            unsigned int length, int restart, int last);
};

static int elite_wait_i2c_ready(struct elite_i2c_priv *priv)
{
    int ret, cnt;

    if (priv->master == 0)
        return 0;

    DECLARE_WAITQUEUE(wait, current);
    add_wait_queue(&priv->wait, &wait);

    ret = 0;
    cnt = 0;
    while(1) {
        if ((readw(priv->regs + STAT_REG) &  I2C_STATUS_MASK) == I2C_READY)
            break;

        set_current_state(TASK_INTERRUPTIBLE);
        if (signal_pending(current))
            break;

         schedule_timeout(1);
         cnt++;

         if (cnt > 50) {
             ret = (-EBUSY);
             break;
         }
    }

    set_current_state(TASK_RUNNING);
    remove_wait_queue(&priv->wait, &wait);

    return ret;
}

static int elite_write_msg(struct elite_i2c_priv *priv, unsigned int slave_addr, char *buf, 
                                    unsigned int length, int restart, int last)
{
    unsigned short tcr_value;
    unsigned int xfer_length;
    int is_timeout;
    int ret = 0;
    int wait_event_result = 0;
    int cnt = 0;

    if (priv->master == 0)
        return -ENXIO;

    if (length < 0)
        return -1;
    xfer_length = 0;

    if (restart == 0)
        ret = elite_wait_i2c_ready(priv);
    if (ret < 0)
        return ret;

    priv->isr_stat.isr_nack        = 0;
    priv->isr_stat.isr_byte_end    = 0;
    priv->isr_stat.isr_timeout     = 0;
    priv->isr_stat.isr_int_pending = 0;

    if (length == 0)
        writew(0, priv->regs + DATAIO_REG);
    else
        writew((unsigned short)(buf[xfer_length] & I2C_CDR_DATA_WRITE_MASK),
                              priv->regs + DATAIO_REG);

    if (restart == 0) {
        writew(readw(priv->regs + CTRL_REG) & (~(I2C_CR_TX_END)), priv->regs + CTRL_REG);
        writew(readw(priv->regs + CTRL_REG) | I2C_CR_CPU_RDY, priv->regs + CTRL_REG);
    }

    tcr_value = 0;
    if (priv->i2c_mode == I2C_STANDARD_MODE) {
        tcr_value = (unsigned short)(I2C_TCR_STANDARD_MODE | I2C_TCR_MASTER_WRITE | \
                                     (slave_addr & I2C_TCR_SLAVE_ADDR_MASK));       
    }
    else if (priv->i2c_mode == I2C_FAST_MODE) {
        tcr_value = (unsigned short)(I2C_TCR_FAST_MODE | I2C_TCR_MASTER_WRITE | \
                                     (slave_addr & I2C_TCR_SLAVE_ADDR_MASK));
    }
    writew(tcr_value, priv->regs + XFER_REG);

    if (restart == 1)
        writew(readw(priv->regs + CTRL_REG) | I2C_CR_CPU_RDY, priv->regs + CTRL_REG);

    ret = 0;
    for (;;) {
        is_timeout = 0;
        wait_event_result = wait_event_interruptible_timeout(priv->wait, 
                                               priv->isr_stat.isr_int_pending,
                                                          (MAX_RX_TIMEOUT * HZ/1000));
        if (likely(wait_event_result > 0))
            ret = 0;
        else if (likely(priv->isr_stat.isr_int_pending == 0)) {
            if (wait_event_result == -ERESTARTSYS) {
                cnt = 0; 
                while(!priv->isr_stat.isr_int_pending) {
                    udelay(10);
                    cnt++;
                    if(cnt > 50) {
                        is_timeout = 1;
                        ret = -ETIMEDOUT;
                        break;
                    }
                }
            } else {
                is_timeout = 1;
                ret = -ETIMEDOUT;
            }
        }

        if (priv->isr_stat.isr_nack == 1) {
            ret = -EIO;
            break;
        }
       
        if (priv->isr_stat.isr_timeout == 1) {
            ret = -ETIMEDOUT;
            break;
        }

        if (is_timeout == 1) {
            ret = -ETIMEDOUT;
            break;
        }

        if (priv->isr_stat.isr_byte_end == 1)
            ++xfer_length;

        priv->isr_stat.isr_int_pending = 0;
        priv->isr_stat.isr_nack        = 0;
        priv->isr_stat.isr_byte_end    = 0;
        priv->isr_stat.isr_timeout     = 0;

        if ((readw(priv->regs + STAT_REG) & I2C_CSR_RCV_ACK_MASK) == I2C_CSR_RCV_NOT_ACK) {
            ret = -EIO;
            break;
        }

        if (length == 0) {
            writew((unsigned short)(I2C_CR_TX_END | I2C_CR_CPU_RDY | I2C_CR_ENABLE), priv->regs + CTRL_REG);
            break;
        }

        if (length > xfer_length) {
            writew((unsigned short)(buf[xfer_length] & I2C_CDR_DATA_WRITE_MASK), priv->regs + DATAIO_REG);
            writew((unsigned short)(I2C_CR_CPU_RDY | I2C_CR_ENABLE), priv->regs + CTRL_REG);
        } else if (length == xfer_length) {
            if (last == 1) {
                writew((I2C_CR_TX_END | I2C_CR_CPU_RDY | I2C_CR_ENABLE), priv->regs + CTRL_REG);
                break;
            } else {
                writew(I2C_CR_ENABLE, priv->regs + CTRL_REG);
                break;
            }
        } else {
            ret = -EIO;
            break;
        }
    }

    return ret;
}

static int elite_read_msg(struct elite_i2c_priv *priv, unsigned int slave_addr, char *buf,
                                  unsigned int length, int restart, int last)
{
    unsigned short tcr_value;
    unsigned int xfer_length;
    int is_timeout;
    int ret = 0;
    int wait_event_result = 0;
    int cnt = 0;

    if (priv->master == 0)
        return -ENXIO;

    if (length <= 0)
        return -1;

    xfer_length = 0;

    if (restart == 0)
        ret = elite_wait_i2c_ready(priv);
    if (ret < 0)
        return ret;

    priv->isr_stat.isr_nack        = 0;
    priv->isr_stat.isr_byte_end    = 0;
    priv->isr_stat.isr_timeout     = 0;
    priv->isr_stat.isr_int_pending = 0;

    writew(readw(priv->regs + CTRL_REG) & ~(I2C_CR_TX_END), priv->regs + CTRL_REG);
    writew(readw(priv->regs + CTRL_REG) & ~(I2C_CR_TX_NEXT_NO_ACK), priv->regs + CTRL_REG);
    if (restart == 0)
        writew(readw(priv->regs + CTRL_REG) | I2C_CR_CPU_RDY, priv->regs + CTRL_REG);
    
    tcr_value = 0;
    if (priv->i2c_mode == I2C_STANDARD_MODE) {
        tcr_value = (unsigned short)(I2C_TCR_STANDARD_MODE | I2C_TCR_MASTER_READ | \
                                     (slave_addr & I2C_TCR_SLAVE_ADDR_MASK));       
    } else if (priv->i2c_mode == I2C_FAST_MODE) {
        tcr_value = (unsigned short)(I2C_TCR_FAST_MODE | I2C_TCR_MASTER_READ | \
                                     (slave_addr & I2C_TCR_SLAVE_ADDR_MASK));
    }
    if (length == 1)
        writew(readw(priv->regs + CTRL_REG) | I2C_CR_TX_NEXT_NO_ACK, priv->regs + CTRL_REG);

    writew(tcr_value, priv->regs + XFER_REG);

    if (restart == 1)
        writew(readw(priv->regs + CTRL_REG) | I2C_CR_CPU_RDY, priv->regs + CTRL_REG);

    ret = 0;
    for (;;) {
        is_timeout = 0;
        wait_event_result = wait_event_interruptible_timeout(priv->wait, priv->isr_stat.isr_int_pending,
                                                          (MAX_RX_TIMEOUT * HZ/1000));
        if (likely(wait_event_result > 0))
            ret = 0;
        else if (likely(priv->isr_stat.isr_int_pending == 0)) {
            if (wait_event_result == -ERESTARTSYS) {
                cnt = 0; 
                while(!priv->isr_stat.isr_int_pending) {
                    udelay(10);
                    cnt++;
                    if(cnt > 50) {
                        is_timeout = 1;
                        ret = -ETIMEDOUT;
                        break;
                    }
                }
            } else {
                is_timeout = 1;
                ret = -ETIMEDOUT;
            }
        }

        if (priv->isr_stat.isr_nack == 1) {
            ret = -EIO;
            break;
        }
       
        if (priv->isr_stat.isr_timeout == 1) {
            ret = -ETIMEDOUT;
            break;
        }

        if (is_timeout == 1) {
            ret = -ETIMEDOUT;
            break;
        }

        if (priv->isr_stat.isr_byte_end == 1) {
            buf[xfer_length] = (readw(priv->regs + DATAIO_REG) >> 8);
            ++xfer_length;
        }

        priv->isr_stat.isr_int_pending = 0;
        priv->isr_stat.isr_nack        = 0;
        priv->isr_stat.isr_byte_end    = 0;
        priv->isr_stat.isr_timeout     = 0;

        if (length > xfer_length) {
            if ((length - 1) == xfer_length)
                writew(readw(priv->regs + CTRL_REG) | (I2C_CR_TX_NEXT_NO_ACK | I2C_CR_CPU_RDY),
                                priv->regs + CTRL_REG);
            else
                writew(readw(priv->regs + CTRL_REG) | I2C_CR_CPU_RDY,
                                priv->regs + CTRL_REG);
        } else if (length == xfer_length) {
            if (last == 1)
                break;
            else
                break;
        } else {
            ret = -EIO;
            break;
        }
    }

    return ret;
}

static int elite_i2c_valid_messages(struct i2c_msg msgs[], int num)
{
    int i;

    if (num < 1 || num > MAX_MESSAGES)
        return -EINVAL;

    for (i = 0; i < num; i++) {
        if (&msgs[i] == NULL)
            return -EINVAL;
        else {
            if (msgs[i].buf == NULL)
                return -EINVAL;
        }
    }

    return 0;
}

static int elite_i2c_doxfer(struct elite_i2c_priv *priv, 
                                struct i2c_msg *msg, int num)
{
    int i, last, restart, ret = 0;
    struct i2c_msg *pmsg;

    for (i = 0; i < 10; ++i)
        ;

    for (i = 0; ret >= 0 && i < num; i++) {
        last = ((i + 1) == num);
        restart = (i != 0);
        
        pmsg = &msg[i];

       if (pmsg->flags & I2C_M_NOSTART)
           restart = 1;

       if (pmsg->flags & I2C_M_RD)
           ret = priv->read_msg(priv, pmsg->addr, pmsg->buf, pmsg->len, restart, last);
       else
           ret = priv->write_msg(priv, pmsg->addr, pmsg->buf, pmsg->len, restart, last);
    }

    if (ret < 0)
        return ret;
    else 
        return i;
}


static int elite_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
    struct elite_i2c_priv *priv = (struct elite_i2c_priv *)adap->algo_data;
    int retry;
    int ret;

    ret = elite_i2c_valid_messages(msgs, num);
    if (ret)
        return ret;

    for (retry = 0; retry < adap->retries; retry++) {
        ret = elite_i2c_doxfer(priv, msgs, num);
        if (ret != -EAGAIN)
            return ret;

        udelay(100);    
    }

    return -EREMOTEIO;    
}

static u32 elite_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

struct i2c_algorithm elite_i2c_algorithm = {
    .master_xfer = elite_i2c_xfer,
    .functionality = elite_i2c_func,
};

static irqreturn_t elite_i2c_irq_handler(int irqno, void *dev_info)
{
    struct elite_i2c_priv *priv = (struct elite_i2c_priv *)dev_info;

    int  wakeup;
    unsigned short isr_status;
    unsigned long flags;
    unsigned short tmp;

    wakeup = 0;

    isr_status = readw(priv->regs + INT_STAT_REG);
    
    if (isr_status & I2C_ISR_NACK_ADDR) {
        writew(I2C_ISR_NACK_ADDR_WRITE_CLEAR, priv->regs + INT_STAT_REG);
        tmp = readw(priv->regs + STAT_REG);
        priv->isr_stat.isr_nack = 1;   
        wakeup = 1;    
    }

    if (isr_status & I2C_ISR_BYTE_END) {
        writew(I2C_ISR_BYTE_END_WRITE_CLEAR, priv->regs + INT_STAT_REG);
        priv->isr_stat.isr_byte_end = 1;
        wakeup = 1;
    }

    if (isr_status & I2C_ISR_SCL_TIME_OUT) {
        writew(I2C_ISR_SCL_TIME_OUT_WRITE_CLEAR, priv->regs + INT_STAT_REG); 
        priv->isr_stat.isr_timeout = 1;
        wakeup = 1;
    }

    if (wakeup) {
        spin_lock_irqsave(&priv->lock, flags);
        priv->isr_stat.isr_int_pending = 1;
        spin_unlock_irqrestore(&priv->lock, flags);

        wake_up_interruptible(&priv->wait);
    }

    return IRQ_HANDLED;
}

static void elite_i2c_adapter_init(struct elite_i2c_priv *priv)
{
    unsigned short tmp;
    /*
     * hard code I2C controller to operate in standard mode
     */
    priv->i2c_mode = I2C_STANDARD_MODE;

    priv->isr_stat.isr_nack        = 0;
    priv->isr_stat.isr_byte_end    = 0;
    priv->isr_stat.isr_timeout     = 0;
    priv->isr_stat.isr_int_pending = 0;

    writew(0, priv->regs + CTRL_REG);
    writew(0, priv->regs + XFER_REG);

    writew(I2C_ISR_ALL_WRITE_CLEAR, priv->regs + INT_STAT_REG);
    writew(I2C_IMR_ALL_ENABLE, priv->regs + INT_MARK_REG);
    writew(I2C_CR_ENABLE, priv->regs + CTRL_REG);

    /* read clear */
    tmp = readw(priv->regs + STAT_REG);

    writew(I2C_ISR_ALL_WRITE_CLEAR, priv->regs + INT_STAT_REG);
    
    /* 
     * only standard mode supported
     * It can also be set as fast mode.
     */
    writew(I2C_TR_STD_VALUE, priv->regs + TIME_REG);
}

#define ELITE_I2C_MAX_DEV_ID 8

struct elite_i2c_priv *elite_i2c[ELITE_I2C_MAX_DEV_ID] = {0};

int elite_i2c_xfer_by_dev_id(unsigned int dev_id, struct i2c_msg *msg, int num)
{
    struct elite_i2c_priv *priv = 0;
    
    if(dev_id >= ELITE_I2C_MAX_DEV_ID || dev_id < 0)
        return -1;    

    priv = elite_i2c[dev_id];
    if(!priv)
        return -1;

    return elite_i2c_doxfer(priv, msg, num);
}

EXPORT_SYMBOL(elite_i2c_xfer_by_dev_id);

/*
 * elite_i2c_probe executes when i2c_bus_type finds a new 
 * platform_device. There are 2 i2c adapters in WM8650, 
 * elite_i2c_probe will execute for 2 times.
 */
static int elite_i2c_probe(struct platform_device *pdev)
{
    struct elite_i2c_priv *i2c;
    struct resource *res;
    int ret = 0;
	int index;

    i2c = kzalloc(sizeof(struct elite_i2c_priv), GFP_KERNEL);
    if (!i2c) {
        dev_err(&pdev->dev, "no memory for elite i2c private data\n");
        return -ENOMEM;
    }
    memset(i2c, 0, sizeof(*i2c));

	ret = of_property_read_u32(pdev->dev.of_node,
				   "linux,i2c-index", &index);
	if(!ret) {
		pdev->id = index;
	}

    i2c->master = 1;
    i2c->dev_id = pdev->id;

    snprintf(i2c->adap.name, sizeof(i2c->adap.name), "elite-i2c-%d", pdev->id);
    i2c->adap.owner    = THIS_MODULE;
    i2c->adap.algo     = &elite_i2c_algorithm;
    i2c->adap.retries  = 2;
    i2c->adap.class    = I2C_CLASS_HWMON | I2C_CLASS_SPD;

    i2c->dev = &pdev->dev;

    spin_lock_init(&i2c->lock);
    init_waitqueue_head(&i2c->wait);


	i2c->clk = clk_get(&pdev->dev, "i2c");
	if (IS_ERR(i2c->clk)) {
		dev_err(&pdev->dev, "cannot get clock\n");
		ret = PTR_ERR(i2c->clk);
		goto err_noclk;
	}

	clk_enable(i2c->clk);


    /* get device io memory resource */
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res) {
        dev_err(&pdev->dev, "cannot find IO resource");
        ret = -ENOENT;
        goto err_res;
    }

    i2c->ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
    if (!i2c->ioarea) {
        dev_err(&pdev->dev, "cannot request mem\n");
        ret = -ENXIO;
        goto err_res;
    }

    i2c->regs = ioremap(res->start, resource_size(res));
    if (!i2c->regs) {
        dev_err(&pdev->dev, "cannot map IO\n");
        ret = -ENXIO;
        goto err_ioarea;
    }
    
    i2c->adap.algo_data  = i2c;
    i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = of_node_get(pdev->dev.of_node);
 
    /*
     * The initializations differ between adapter 0
     * and adapter 1.
     */
    elite_i2c_adapter_init(i2c);

    /* get device irq resource */ 
    i2c->irq = ret = platform_get_irq(pdev, 0);
    if (ret <= 0) {
        dev_err(&pdev->dev, "cannot find IRQ");
        goto err_ioremap;
    }

    ret = request_irq(i2c->irq, elite_i2c_irq_handler, IRQF_DISABLED, 
                       dev_name(&pdev->dev), i2c);
    if (ret != 0) {
        dev_err(&pdev->dev, "cannot claim IRQ %d\n", i2c->irq);
        goto err_ioremap;
    }

    i2c->adap.nr = pdev->id;

    i2c->read_msg  = elite_read_msg;
    i2c->write_msg = elite_write_msg;
    
    ret = i2c_add_numbered_adapter(&i2c->adap);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to add bus to i2c core\n");
        goto err_irq;
    }

    platform_set_drvdata(pdev, i2c);
    
    elite_i2c[pdev->id] = i2c;

	of_i2c_register_devices(&i2c->adap);
    
    dev_info(&pdev->dev, "S3G elite SoC I2C adapter (nr = %d) registered\n", pdev->id);

    return 0;

err_irq:
    free_irq(i2c->irq, i2c);

err_ioremap:
    iounmap(i2c->regs);

err_ioarea:
    release_resource(i2c->ioarea);
    kfree(i2c->ioarea);

err_res:
	clk_disable(i2c->clk);
	clk_put(i2c->clk);
err_noclk:
    kfree(i2c);
    return ret;
}

static int elite_i2c_remove(struct platform_device *pdev)
{
    struct elite_i2c_priv *i2c = platform_get_drvdata(pdev);

    i2c_del_adapter(&i2c->adap);
    free_irq(i2c->irq, i2c);

	clk_disable(i2c->clk);
	clk_put(i2c->clk);

    iounmap(i2c->regs);

    release_resource(i2c->ioarea);
    kfree(i2c->ioarea);
    kfree(i2c);

    return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elite_i2c_suspend(struct device *dev)
{
	struct elite_i2c_priv *i2c = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c->adap);
	i2c->is_suspended = true;
	i2c_unlock_adapter(&i2c->adap);

	return 0;
}

static int elite_i2c_resume(struct device *dev)
{
	struct elite_i2c_priv *i2c = dev_get_drvdata(dev);

	i2c_lock_adapter(&i2c->adap);
	elite_i2c_adapter_init(i2c);
	i2c->is_suspended = false;
	i2c_unlock_adapter(&i2c->adap);

	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int elite_i2c_runtime_suspend(struct device *dev)
{
	return 0;
}
static int elite_i2c_runtime_resume(struct device *dev)
{
	return 0;
}
#endif


static struct platform_device_id elite_i2c_driver_ids[] = {
    {
        .name  = "elite-i2c",
		.driver_data	= (kernel_ulong_t)NULL,
    }, {
        .name  = "elite-i2c.0",
		.driver_data	= (kernel_ulong_t)NULL,
    }, {
        .name  = "elite-i2c.1",
		.driver_data	= (kernel_ulong_t)NULL,
    }, 
	{ /* sentinel */ },
};

static const struct dev_pm_ops elite_i2c_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(elite_i2c_suspend, elite_i2c_resume)
	SET_RUNTIME_PM_OPS(elite_i2c_runtime_suspend,
				elite_i2c_runtime_resume, NULL)
};

#ifdef CONFIG_OF
static const struct of_device_id elite_i2c_match[] = {
	{ .compatible = "s3graphics,elite1000-i2c" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, elite_i2c_match);
#endif

static struct platform_driver elite_i2c_driver = {
    .probe     = elite_i2c_probe,
    .remove    = elite_i2c_remove,
    .id_table  = elite_i2c_driver_ids,
    .driver    = {
        .owner = THIS_MODULE,
        .name  = "s3graphics-elite-i2c",
        .pm	= &elite_i2c_dev_pm_ops,
		.of_match_table = of_match_ptr(elite_i2c_match),
    },
};

static int __init elite_i2c_init(void)
{
    return platform_driver_register(&elite_i2c_driver);
}
subsys_initcall(elite_i2c_init);

static void __exit elite_i2c_exit(void)
{
    platform_driver_unregister(&elite_i2c_driver);
}
module_exit(elite_i2c_exit);


MODULE_AUTHOR("S3 Graphics Co.,Ltd");
MODULE_DESCRIPTION("Elite I2C Bus Driver");
MODULE_LICENSE("GPL");

