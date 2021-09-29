void trng_init_32k(void);
void trng_init_128k(void);
void trng_deinit(void);
void trng_load_setting(hal_trng_adapter_t *ptrng_adp, hal_rng_st_reg_t *ptrng_st_reg,  hal_rng_reg_t *ptrng_reg);
void trng_run_32k(uint32_t length, uint32_t *arr);
void trng_run_128k(uint32_t length, uint32_t *arr);



