#ifndef __ASSEMBLY__
extern void *elite_context_area;

void __cortex_a9_save(unsigned int restore_address,unsigned int mode);
void __cortex_a9_restore(void);
void __shut_off_mmu(void);
void elite_pg_startup(void);
#endif //__ASSEMBLY__

