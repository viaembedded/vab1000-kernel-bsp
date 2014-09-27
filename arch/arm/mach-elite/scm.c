#include <linux/types.h>
#include <linux/slab.h>
#include "scm_handle.h"
#include <asm/cacheflush.h>
#include <linux/module.h>
#include <mach/iomap.h>
#include <linux/io.h>
#define CACHELINESIZE 32

/*alloc_scm_command() - Allocate an SCM command
 * @cmd_size: size of the command buffer
 * @resp_size: size of the response buffer
 *
 * Allocate an SCM command, including enough room for the command
 * and response headers as well as the command and response buffers.
 *
 * Returns a valid &scm_command on success or %NULL if the allocation fails.
 */

static struct scm_command *alloc_scm_command(size_t cmd_size, size_t resp_size)
{
	struct scm_command *cmd;
	struct scm_response *rsp;
	size_t len = sizeof(*cmd) + sizeof(struct scm_response) + cmd_size +
		resp_size;

	len = (len + CACHELINESIZE - 1) & (~(CACHELINESIZE -1));

	cmd = kzalloc(len, GFP_KERNEL);
	if (cmd) {
		cmd->len = len;
		cmd->buf_offset = offsetof(struct scm_command, buf);
		cmd->resp_hdr_offset = cmd->buf_offset + cmd_size;
		
		rsp = (struct scm_response*)((void*)cmd + cmd->resp_hdr_offset);
		rsp->len = resp_size;
		rsp->buf_offset = sizeof(*rsp);
	}
	return cmd;
}

/**
 * free_scm_command() - Free an SCM command
 * @cmd: command to free
 *
 * Free an SCM command.
 */
static inline void free_scm_command(struct scm_command *cmd)
{
	kfree((void*)cmd);
}

/**
 * scm_command_to_response() - Get a pointer to a scm_response
 * @cmd: command
 *
 * Returns a pointer to a response for a command.
 */
static inline struct scm_response *scm_command_to_response(
		const struct scm_command *cmd)
{
	return (void *)cmd + cmd->resp_hdr_offset;
}

/**
 * scm_get_command_buffer() - Get a pointer to a command buffer
 * @cmd: command
 *
 * Returns a pointer to the command buffer of a command.
 */

static inline void *scm_get_command_buffer(const struct scm_command *cmd)
{
	return (void *)cmd->buf;
}

/**
 * scm_get_response_buffer() - Get a pointer to a response buffer
 * @rsp: response
 *
 * Returns a pointer to a response buffer of a response.
 */
static inline void *scm_get_response_buffer(const struct scm_response *rsp)
{
	return (void *)rsp + rsp->buf_offset;
}

static int scm_remap_error(int err)
{
	switch (err) {
	case SCM_ERROR:
		return -EIO;
	case SCM_EINVAL_ADDR:
	case SCM_EINVAL_ARG:
		return -EINVAL;
	case SCM_EOPNOTSUPP:
		return -EOPNOTSUPP;
	case SCM_ENOMEM:
		return -ENOMEM;
	}
	return -EINVAL;
}

static u32 smc(u32 cmd_addr)
{
	int context_id;
	register u32 r0 asm("r0") = 0;
	register u32 r1 asm("r1") = (u32)&context_id;
	register u32 r2 asm("r2") = cmd_addr;

	do {
		asm volatile(
			__asmeq("%0", "r0")
			__asmeq("%1", "r0")
			__asmeq("%2", "r1")
			__asmeq("%3", "r2")
			"push   {lr} \r\n"
			"mrs lr, spsr \r\n"
			"push   {lr} \r\n"
			"smc	#0	@ switch to secure world\r\n"
			"pop    {lr} \r\n"
			"msr spsr, lr \r\n"
			"pop    {lr}"
			: "=r" (r0)
			: "r" (r0), "r" (r1), "r" (r2)
			: "r3");
	} while(r0 == SCM_INTERRUPTED);

	return r0;
}

static int __scm_call(const struct scm_command *cmd)
{
	int ret;
	u32 cmd_addr = virt_to_phys((void*)cmd);

	ret = smc(cmd_addr);
	if (ret < 0)
		ret = scm_remap_error(ret);

	return ret;
}

/**
 * scm_call() - Send an SCM command
 * @svc_id: service identifier
 * @cmd_id: command identifier
 * @cmd_buf: command buffer
 * @cmd_len: length of the command buffer
 * @resp_buf: response buffer
 * @resp_len: length of the response buffer
 *
 * Sends a command to the SCM and waits for the command to finish processing.
 */
int scm_call(u32 svc_id, u32 cmd_id, const void *cmd_buf, size_t cmd_len,
		void *resp_buf, size_t resp_len)
{
	int ret;
	struct scm_command *cmd;
	struct scm_response *rsp;

	cmd = alloc_scm_command(cmd_len, resp_len);
	if (!cmd)
		return -ENOMEM;
	cmd->id = (svc_id << 10) | cmd_id;
	if (cmd_buf)
		memcpy(scm_get_command_buffer(cmd), cmd_buf, cmd_len);


	ret = __scm_call(cmd);


	if (ret)
		goto out;

	if (!resp_buf)
		goto out;

	rsp = scm_command_to_response(cmd);
/*
	do {
		u32 start = (u32)rsp;
		u32 end = (u32)scm_get_response_buffer(rsp) + resp_len;
		start &= ~(CACHELINESIZE - 1);
		while (start < end) {
			asm ("mcr p15, 0, %0, c7, c6, 1" : : "r" (start)
			     : "memory");
			start += CACHELINESIZE;
		}
	} while (!rsp->is_complete);
*/

	if (resp_buf)
		memcpy(resp_buf, scm_get_response_buffer(rsp), resp_len);
out:
	free_scm_command(cmd);
	return ret;
}

int svc_system_enable_l2cache(void)
{
	u32 buffer[64] = {0};
	struct scm_command *cmd;
	struct scm_response *rsp;

	cmd = (struct scm_command*)buffer;
	cmd->len = sizeof(struct scm_response) + sizeof(*cmd);
	cmd->buf_offset = offsetof(struct scm_command, buf);
	cmd->resp_hdr_offset = cmd->buf_offset;
		
	rsp = (struct scm_response*)((void*)cmd + cmd->resp_hdr_offset);
	rsp->len = 0;
	rsp->buf_offset = sizeof(*rsp);

	cmd->id = (SVC_SYSTEM << 10) | SVC_SYSTEM_ENABLE_L2CACHE;
	
	return __scm_call(cmd);
	//return scm_call(SVC_SYSTEM, SVC_SYSTEM_ENABLE_L2CACHE, NULL, 0, NULL, 0);
}

int svc_system_boot_secondary(int cpu, u32 address)
{
	cmd_boot_secondary_t cmd;
	cmd.cpuid = cpu;
	cmd.address =address;

	return scm_call(SVC_SYSTEM, SVC_SYSTEM_BOOT_SECONDARY, &cmd, sizeof(cmd), NULL, 0);
}

int svc_system_cpu_pg(u32 address)
{
	/* use stack memory as command to avoid memory leak since CPU will return here again! */
	u32 buffer[64] = {0};
	struct scm_command *cmd;
	struct scm_response *rsp;

	cmd = (struct scm_command*)buffer;
	cmd->len = sizeof(struct scm_response) + sizeof(*cmd) + sizeof(u32);
	cmd->buf_offset = offsetof(struct scm_command, buf);
	cmd->resp_hdr_offset = cmd->buf_offset + sizeof(u32);
		
	rsp = (struct scm_response*)((void*)cmd + cmd->resp_hdr_offset);
	rsp->len = 0;
	rsp->buf_offset = sizeof(*rsp);

	cmd->id = (SVC_SYSTEM << 10) | SVC_SYSTEM_CPU_PG;

	*(u32*)cmd->buf = address;
	
	return __scm_call(cmd);

	//return scm_call(SVC_SYSTEM, SVC_SYSTEM_CPU_PG, &address, sizeof(u32), NULL, 0);
}

int svc_system_enter_stm(u32 address)
{
	/* use stack memory as command to avoid memory leak since CPU will return here again! */
	u32 buffer[64] = {0};
	struct scm_command *cmd;
	struct scm_response *rsp;

	cmd = (struct scm_command*)buffer;
	cmd->len = sizeof(struct scm_response) + sizeof(*cmd) + sizeof(u32);
	cmd->buf_offset = offsetof(struct scm_command, buf);
	cmd->resp_hdr_offset = cmd->buf_offset + sizeof(u32);
		
	rsp = (struct scm_response*)((void*)cmd + cmd->resp_hdr_offset);
	rsp->len = 0;
	rsp->buf_offset = sizeof(*rsp);

	cmd->id = (SVC_SYSTEM << 10) | SVC_SYSTEM_ENYER_STM;

	*(u32*)cmd->buf = address;
	
	return __scm_call(cmd);

	//return scm_call(SVC_SYSTEM, SVC_SYSTEM_CPU_PG, &address, sizeof(u32), NULL, 0);
}

int svc_system_get_boot_vector(int cpu, unsigned int *address)
{
	int ret;
	ret = scm_call(SVC_SYSTEM, SVC_SYSTEM_GET_BOOT_VECTOR, &cpu, sizeof(int), address, sizeof(u32));
	return ret;
}

EXPORT_SYMBOL(svc_system_get_boot_vector);

unsigned int svc_system_read_pmu_reg(unsigned int reg)
{
	cmd_prog_pmu_reg_t cmd;
	cmd.read = 1;
        cmd.reg = reg;
	scm_call(SVC_SYSTEM, SVC_SYSTEM_PROG_PMU_REG, &cmd, sizeof(cmd), &cmd.value, sizeof(unsigned int));
	return cmd.value;
}

void svc_system_write_pmu_reg(unsigned int reg, unsigned int value)
{
	cmd_prog_pmu_reg_t cmd;
	cmd.read = 0;
	cmd.reg = reg;
	cmd.value = value;
	scm_call(SVC_SYSTEM, SVC_SYSTEM_PROG_PMU_REG, &cmd, sizeof(cmd), NULL, 0);
}

EXPORT_SYMBOL(svc_system_read_pmu_reg);
EXPORT_SYMBOL(svc_system_write_pmu_reg);

int svc_system_is_secureboot(void)
{
	/* this register is programmed in secure os*/
	return readl(IO_ADDRESS(ELITE_SECUREBOOT_REG));
}

EXPORT_SYMBOL(svc_system_is_secureboot);

