#ifndef __SCM_HANDLE_H__
#define __SCM_HANDLE_H__


#define SCM_ENOMEM        -5
#define SCM_EOPNOTSUPP        -4
#define SCM_EINVAL_ADDR        -3
#define SCM_EINVAL_ARG        -2
#define SCM_ERROR        -1
#define SCM_OK            0
#define SCM_INTERRUPTED        1

#ifndef __ASSEMBLY__

/* service identifier */
#define SCM_SVC_BOOT        0x1
#define SCM_SVC_PIL            0x2

/* command identifier */
#define SCM_BOOT_ADDR            0x1
#define SCM_FLAG_COLDBOOT_CPU1    0x1
#define SCM_FLAG_WARMBOOT_CPU1    0x2
#define SCM_FLAG_WARMBOOT_CPU0    0x4

/* All the services definitions */
#define SVC_SYSTEM 0

/* All command definition for system service */
#define SVC_SYSTEM_ENABLE_L2CACHE 0
#define SVC_SYSTEM_BOOT_SECONDARY 1
#define SVC_SYSTEM_CPU_PG         2

#define SVC_SYSTEM_SET_SECURE_VAR 3
#define SVC_SYSTEM_GET_SECURE_VAR 4


#define SVC_SYSTEM_GET_BOOT_VECTOR 5

#define SVC_SYSTEM_PROG_PMU_REG    7
#define SVC_SYSTEM_ENYER_STM       8

/**
 * struct scm_command - one SCM command buffer
 * @len: total available memory for command and response
 * @buf_offset: start of command buffer
 * @resp_hdr_offset: start of response buffer
 * @id: command to be executed
 * @buf: buffer returned from scm_get_command_buffer()
 *
 * An SCM command is laid out in memory as follows:
 *
 *    ------------------- <--- struct scm_command
 *    | command header  |
 *    ------------------- <--- scm_get_command_buffer()
 *    | command buffer  |
 *    ------------------- <--- struct scm_response and
 *    | response header |      scm_command_to_response()
 *    ------------------- <--- scm_get_response_buffer()
 *    | response buffer |
 *    -------------------
 *
 * There can be arbitrary padding between the headers and buffers so
 * you should always use the appropriate scm_get_*_buffer() routines
 * to access the buffers in a safe manner.
 */
struct scm_command {
    u32    len;
    u32    buf_offset;
    u32    resp_hdr_offset;
    u32    id;
    u32    buf[0];
};

/**
 * struct scm_response - one SCM response buffer
 * @len: total available memory for response
 * @buf_offset: start of response data relative to start of scm_response
 * @is_complete: indicates if the command has finished processing
 */
struct scm_response {
    u32    len;
    u32    buf_offset;
    u32    is_complete;
};


/* data structure definitions */

typedef struct cmd_boot_secondary {
    int cpuid;
    u32 address;
}cmd_boot_secondary_t;

typedef struct cmd_set_sec_var {
    char *name;
    int  len;
    int  value;
}cmd_set_sec_var_t;

typedef struct cmd_get_sec_var {
    char *name;
    int len;
}cmd_get_sec_var_t;

typedef struct cmd_prog_pmu_reg {
    int read;
    unsigned int reg;
    unsigned int value;
}cmd_prog_pmu_reg_t;

extern int svc_system_enter_stm(u32 address);

#endif //__ASSEMBLY__ 

#endif //__SCM_HANDLE_H__
