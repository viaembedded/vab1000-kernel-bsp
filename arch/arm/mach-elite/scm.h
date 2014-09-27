#ifndef __SCM_H__
#define __SCM_H__
extern int svc_system_is_secureboot(void);
extern int svc_system_enable_l2cache(void);
extern int svc_system_boot_secondary(int cpu, unsigned int address);
extern int svc_system_cpu_pg(unsigned int address);
extern int svc_system_get_boot_vector(int cpu, unsigned int *address);
#endif
