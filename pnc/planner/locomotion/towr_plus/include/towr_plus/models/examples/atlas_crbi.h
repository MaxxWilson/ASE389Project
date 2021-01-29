/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int atlas_crbi(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int atlas_crbi_alloc_mem(void);
int atlas_crbi_init_mem(int mem);
void atlas_crbi_free_mem(int mem);
int atlas_crbi_checkout(void);
void atlas_crbi_release(int mem);
void atlas_crbi_incref(void);
void atlas_crbi_decref(void);
casadi_int atlas_crbi_n_out(void);
casadi_int atlas_crbi_n_in(void);
casadi_real atlas_crbi_default_in(casadi_int i);
const char* atlas_crbi_name_in(casadi_int i);
const char* atlas_crbi_name_out(casadi_int i);
const casadi_int* atlas_crbi_sparsity_in(casadi_int i);
const casadi_int* atlas_crbi_sparsity_out(casadi_int i);
int atlas_crbi_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int jac_atlas_crbi(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int jac_atlas_crbi_alloc_mem(void);
int jac_atlas_crbi_init_mem(int mem);
void jac_atlas_crbi_free_mem(int mem);
int jac_atlas_crbi_checkout(void);
void jac_atlas_crbi_release(int mem);
void jac_atlas_crbi_incref(void);
void jac_atlas_crbi_decref(void);
casadi_int jac_atlas_crbi_n_out(void);
casadi_int jac_atlas_crbi_n_in(void);
casadi_real jac_atlas_crbi_default_in(casadi_int i);
const char* jac_atlas_crbi_name_in(casadi_int i);
const char* jac_atlas_crbi_name_out(casadi_int i);
const casadi_int* jac_atlas_crbi_sparsity_in(casadi_int i);
const casadi_int* jac_atlas_crbi_sparsity_out(casadi_int i);
int jac_atlas_crbi_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif