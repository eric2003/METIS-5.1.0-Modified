/*!
\file gk_proto.h
\brief This file contains function prototypes

\date   Started 3/27/2007
\author George
\version\verbatim $Id: gk_proto.h 12591 2012-09-01 19:03:15Z karypis $ \endverbatim
*/

#ifndef _GK_PROTO_H_
#define _GK_PROTO_H_
#include "metis_def.h"
#ifdef __cplusplus
extern "C" {
#endif

/*-------------------------------------------------------------
 * blas.c 
 *-------------------------------------------------------------*/
GK_MKBLAS_PROTO(gk_c,   char,     int)
GK_MKBLAS_PROTO(gk_i,   int,      int)
GK_MKBLAS_PROTO(gk_i32, int32_t,  int32_t)
GK_MKBLAS_PROTO(gk_i64, int64_t,  int64_t)
GK_MKBLAS_PROTO(gk_z,   ssize_t,  ssize_t)
GK_MKBLAS_PROTO(gk_f,   float,    float)
GK_MKBLAS_PROTO(gk_d,   double,   double)
GK_MKBLAS_PROTO(gk_idx, gk_idx_t, gk_idx_t)




/*-------------------------------------------------------------
 * io.c
 *-------------------------------------------------------------*/
METIS_API(FILE*) gk_fopen(char *, char *, const char *);
METIS_API(void) gk_fclose(FILE *);
METIS_API(gk_idx_t) gk_getline(char **lineptr, size_t *n, FILE *stream);
METIS_API(char **) gk_readfile(char *fname, gk_idx_t *r_nlines);
METIS_API(int32_t *) gk_i32readfile(char *fname, gk_idx_t *r_nlines);
METIS_API(int64_t *) gk_i64readfile(char *fname, gk_idx_t *r_nlines);
METIS_API(int32_t *) gk_i32readfilebin(char *fname, ssize_t *r_nelmnts);
METIS_API(int64_t *) gk_i64readfilebin(char *fname, ssize_t *r_nelmnts);
METIS_API(float *) gk_freadfilebin(char *fname, ssize_t *r_nelmnts);
METIS_API(size_t) gk_fwritefilebin(char *fname, size_t n, float *a);
METIS_API(double*) gk_dreadfilebin(char *fname, ssize_t *r_nelmnts);




/*-------------------------------------------------------------
 * fs.c
 *-------------------------------------------------------------*/
METIS_API(int) gk_fexists(char *);
METIS_API(int) gk_dexists(char *);
METIS_API(intmax_t) gk_getfsize(char *);
METIS_API(void) gk_getfilestats(char *fname, size_t *r_nlines, size_t *r_ntokens, 
          size_t *r_max_nlntokens, size_t *r_nbytes);
METIS_API(char*) gk_getbasename(char *path);
METIS_API(char*) gk_getextname(char *path);
METIS_API(char*) gk_getfilename(char *path);
METIS_API(char*) gk_getpathname(char *path);
METIS_API(int) gk_mkpath(char *);
METIS_API(int) gk_rmpath(char *);



/*-------------------------------------------------------------
 * memory.c
 *-------------------------------------------------------------*/
GK_MKALLOC_PROTO(gk_c,   char)
GK_MKALLOC_PROTO(gk_i,   int)
GK_MKALLOC_PROTO(gk_i32, int32_t)
GK_MKALLOC_PROTO(gk_i64, int64_t)
GK_MKALLOC_PROTO(gk_z,   ssize_t)
GK_MKALLOC_PROTO(gk_f,   float)
GK_MKALLOC_PROTO(gk_d,   double)
GK_MKALLOC_PROTO(gk_idx, gk_idx_t)

GK_MKALLOC_PROTO(gk_ckv,   gk_ckv_t)
GK_MKALLOC_PROTO(gk_ikv,   gk_ikv_t)
GK_MKALLOC_PROTO(gk_i32kv, gk_i32kv_t)
GK_MKALLOC_PROTO(gk_i64kv, gk_i64kv_t)
GK_MKALLOC_PROTO(gk_zkv,   gk_zkv_t)
GK_MKALLOC_PROTO(gk_fkv,   gk_fkv_t)
GK_MKALLOC_PROTO(gk_dkv,   gk_dkv_t)
GK_MKALLOC_PROTO(gk_skv,   gk_skv_t)
GK_MKALLOC_PROTO(gk_idxkv, gk_idxkv_t)

METIS_API(void)   gk_AllocMatrix(void***, size_t, size_t, size_t);
METIS_API(void)   gk_FreeMatrix(void***, size_t, size_t);
METIS_API(int)    gk_malloc_init();
METIS_API(void)   gk_malloc_cleanup(int showstats);
METIS_API(void*)  gk_malloc(size_t nbytes, char* msg);
METIS_API(void*)  gk_realloc(void* oldptr, size_t nbytes, char* msg);
METIS_API(void)   gk_free(void** ptr1, ...);
METIS_API(size_t) gk_GetCurMemoryUsed();
METIS_API(size_t) gk_GetMaxMemoryUsed();


/*-------------------------------------------------------------
 * seq.c
 *-------------------------------------------------------------*/
METIS_API(gk_seq_t * ) gk_seq_ReadGKMODPSSM(char *file_name);
METIS_API(gk_i2cc2i_t *) gk_i2cc2i_create_common(char *alphabet);
METIS_API(void) gk_seq_init(gk_seq_t *seq);




/*-------------------------------------------------------------
 * pdb.c
 *-------------------------------------------------------------*/
METIS_API(char) gk_threetoone(char *res);
METIS_API(void) gk_freepdbf(pdbf *p);
METIS_API(pdbf*)gk_readpdbfile(char *fname);
METIS_API(void) gk_writefullatom(pdbf *p, char *fname);
METIS_API(void) gk_writebackbone(pdbf *p, char *fname);
METIS_API(void) gk_writealphacarbons(pdbf *p, char *fname);
METIS_API(void) gk_showcorruption(pdbf *p);


/*-------------------------------------------------------------
 * error.c
 *-------------------------------------------------------------*/
METIS_API(void) gk_set_exit_on_error(int value);
METIS_API(void) errexit(char *,...);
METIS_API(void) gk_errexit(int signum, char *,...);
METIS_API(int) gk_sigtrap();
METIS_API(int) gk_siguntrap();
METIS_API(void) gk_sigthrow(int signum);
METIS_API(void) gk_SetSignalHandlers();
METIS_API(void) gk_UnsetSignalHandlers();
METIS_API(void) gk_NonLocalExit_Handler(int signum);
METIS_API(char*) gk_strerror(int errnum);
METIS_API(void) PrintBackTrace();


/*-------------------------------------------------------------
 * util.c
 *-------------------------------------------------------------*/
METIS_API(void)  gk_RandomPermute(size_t, int *, int);
METIS_API(void)  gk_array2csr(size_t n, size_t range, int *array, int *ptr, int *ind);
METIS_API(int)   gk_log2(int);
METIS_API(int)   gk_ispow2(int);
METIS_API(float) gk_flog2(float);


/*-------------------------------------------------------------
 * time.c
 *-------------------------------------------------------------*/
METIS_API(gk_wclock_t) gk_WClockSeconds(void);
METIS_API(double) gk_CPUSeconds(void);

/*-------------------------------------------------------------
 * string.c
 *-------------------------------------------------------------*/

METIS_API(char*) gk_strchr_replace(char* str, char* fromlist, char* tolist);
METIS_API(int)     gk_strstr_replace(char* str, char* pattern, char* replacement, char* options, char** new_str);
METIS_API(char*) gk_strtprune(char*, char*);
METIS_API(char*) gk_strhprune(char*, char*);
METIS_API(char*) gk_strtoupper(char*);
METIS_API(char*) gk_strtolower(char*);
METIS_API(char*) gk_strdup(char* orgstr);
METIS_API(int)     gk_strcasecmp(char* s1, char* s2);
METIS_API(int)     gk_strrcmp(char* s1, char* s2);
METIS_API(char*) gk_time2str(time_t time);
METIS_API(time_t)  gk_str2time(char* str);
METIS_API(int)     gk_GetStringID(gk_StringMap_t* strmap, char* key);



/*-------------------------------------------------------------
 * sort.c 
 *-------------------------------------------------------------*/
METIS_API(void) gk_csorti(size_t, char *);
METIS_API(void) gk_csortd(size_t, char *);
METIS_API(void) gk_isorti(size_t, int *);
METIS_API(void) gk_isortd(size_t, int *);
METIS_API(void) gk_fsorti(size_t, float *);
METIS_API(void) gk_fsortd(size_t, float *);
METIS_API(void) gk_dsorti(size_t, double *);
METIS_API(void) gk_dsortd(size_t, double *);
METIS_API(void) gk_idxsorti(size_t, gk_idx_t *);
METIS_API(void) gk_idxsortd(size_t, gk_idx_t *);
METIS_API(void) gk_ckvsorti(size_t, gk_ckv_t *);
METIS_API(void) gk_ckvsortd(size_t, gk_ckv_t *);
METIS_API(void) gk_ikvsorti(size_t, gk_ikv_t *);
METIS_API(void) gk_ikvsortd(size_t, gk_ikv_t *);
METIS_API(void) gk_i32kvsorti(size_t, gk_i32kv_t *);
METIS_API(void) gk_i32kvsortd(size_t, gk_i32kv_t *);
METIS_API(void) gk_i64kvsorti(size_t, gk_i64kv_t *);
METIS_API(void) gk_i64kvsortd(size_t, gk_i64kv_t *);
METIS_API(void) gk_zkvsorti(size_t, gk_zkv_t *);
METIS_API(void) gk_zkvsortd(size_t, gk_zkv_t *);
METIS_API(void) gk_fkvsorti(size_t, gk_fkv_t *);
METIS_API(void) gk_fkvsortd(size_t, gk_fkv_t *);
METIS_API(void) gk_dkvsorti(size_t, gk_dkv_t *);
METIS_API(void) gk_dkvsortd(size_t, gk_dkv_t *);
METIS_API(void) gk_skvsorti(size_t, gk_skv_t *);
METIS_API(void) gk_skvsortd(size_t, gk_skv_t *);
METIS_API(void) gk_idxkvsorti(size_t, gk_idxkv_t *);
METIS_API(void) gk_idxkvsortd(size_t, gk_idxkv_t *);


/*-------------------------------------------------------------
 * Selection routines
 *-------------------------------------------------------------*/
METIS_API(int)  gk_dfkvkselect(size_t, int, gk_fkv_t *);
METIS_API(int)  gk_ifkvkselect(size_t, int, gk_fkv_t *);


/*-------------------------------------------------------------
 * Priority queue 
 *-------------------------------------------------------------*/
GK_MKPQUEUE_PROTO(gk_ipq,   gk_ipq_t,   int,      gk_idx_t)
GK_MKPQUEUE_PROTO(gk_i32pq, gk_i32pq_t, int32_t,  gk_idx_t)
GK_MKPQUEUE_PROTO(gk_i64pq, gk_i64pq_t, int64_t,  gk_idx_t)
GK_MKPQUEUE_PROTO(gk_fpq,   gk_fpq_t,   float,    gk_idx_t)
GK_MKPQUEUE_PROTO(gk_dpq,   gk_dpq_t,   double,   gk_idx_t)
GK_MKPQUEUE_PROTO(gk_idxpq, gk_idxpq_t, gk_idx_t, gk_idx_t)


/*-------------------------------------------------------------
 * HTable routines
 *-------------------------------------------------------------*/
METIS_API(gk_HTable_t *)HTable_Create(int nelements);
METIS_API(void)         HTable_Reset(gk_HTable_t *htable);
METIS_API(void)         HTable_Resize(gk_HTable_t *htable, int nelements);
METIS_API(void)         HTable_Insert(gk_HTable_t *htable, int key, int val);
METIS_API(void)         HTable_Delete(gk_HTable_t *htable, int key);
METIS_API(int)          HTable_Search(gk_HTable_t *htable, int key);
METIS_API(int)          HTable_GetNext(gk_HTable_t *htable, int key, int *val, int type);
METIS_API(int)          HTable_SearchAndDelete(gk_HTable_t *htable, int key);
METIS_API(void)         HTable_Destroy(gk_HTable_t *htable);
METIS_API(int)          HTable_HFunction(int nelements, int key);
 

/*-------------------------------------------------------------
 * Tokenizer routines
 *-------------------------------------------------------------*/
METIS_API(void) gk_strtokenize(char *line, char *delim, gk_Tokens_t *tokens);
METIS_API(void) gk_freetokenslist(gk_Tokens_t *tokens);

/*-------------------------------------------------------------
 * Encoder/Decoder
 *-------------------------------------------------------------*/
METIS_API(void) encodeblock(unsigned char *in, unsigned char *out);
METIS_API(void) decodeblock(unsigned char *in, unsigned char *out);
METIS_API(void) GKEncodeBase64(int nbytes, unsigned char *inbuffer, unsigned char *outbuffer);
METIS_API(void) GKDecodeBase64(int nbytes, unsigned char *inbuffer, unsigned char *outbuffer);


/*-------------------------------------------------------------
 * random.c
 *-------------------------------------------------------------*/
GK_MKRANDOM_PROTO(gk_c,   size_t, char)
GK_MKRANDOM_PROTO(gk_i,   size_t, int)
GK_MKRANDOM_PROTO(gk_f,   size_t, float)
GK_MKRANDOM_PROTO(gk_d,   size_t, double)
GK_MKRANDOM_PROTO(gk_idx, size_t, gk_idx_t)
GK_MKRANDOM_PROTO(gk_z,   size_t, ssize_t)
METIS_API(void) gk_randinit(uint64_t);
METIS_API(uint64_t) gk_randint64(void);
METIS_API(uint32_t) gk_randint32(void);


/*-------------------------------------------------------------
 * OpenMP fake functions
 *-------------------------------------------------------------*/
#if !defined(__OPENMP__)
METIS_API(void) omp_set_num_threads(int num_threads);
METIS_API(int) omp_get_num_threads(void);
METIS_API(int) omp_get_max_threads(void);
METIS_API(int) omp_get_thread_num(void);
METIS_API(int) omp_get_num_procs(void);
METIS_API(int) omp_in_parallel(void);
METIS_API(void) omp_set_dynamic(int num_threads);
METIS_API(int) omp_get_dynamic(void);
METIS_API(void) omp_set_nested(int nested);
METIS_API(int) omp_get_nested(void);
#endif /* __OPENMP__ */


/*-------------------------------------------------------------
 * CSR-related functions
 *-------------------------------------------------------------*/
METIS_API(gk_csr_t*) gk_csr_Create();
METIS_API(void) gk_csr_Init(gk_csr_t *mat);
METIS_API(void) gk_csr_Free(gk_csr_t **mat);
METIS_API(void) gk_csr_FreeContents(gk_csr_t *mat);
METIS_API(gk_csr_t*) gk_csr_Dup(gk_csr_t *mat);
METIS_API(gk_csr_t*) gk_csr_ExtractSubmatrix(gk_csr_t *mat, int rstart, int nrows);
METIS_API(gk_csr_t*) gk_csr_ExtractRows(gk_csr_t *mat, int nrows, int *rind);
METIS_API(gk_csr_t*) gk_csr_ExtractPartition(gk_csr_t *mat, int *part, int pid);
METIS_API(gk_csr_t**) gk_csr_Split(gk_csr_t *mat, int *color);
METIS_API(gk_csr_t*) gk_csr_Read(char *filename, int format, int readvals, int numbering);
METIS_API(void) gk_csr_Write(gk_csr_t *mat, char *filename, int format, int writevals, int numbering);
METIS_API(gk_csr_t*) gk_csr_Prune(gk_csr_t *mat, int what, int minf, int maxf);
METIS_API(gk_csr_t*) gk_csr_LowFilter(gk_csr_t *mat, int what, int norm, float fraction);
METIS_API(gk_csr_t*) gk_csr_TopKPlusFilter(gk_csr_t *mat, int what, int topk, float keepval);
METIS_API(gk_csr_t*) gk_csr_ZScoreFilter(gk_csr_t *mat, int what, float zscore);
METIS_API(void) gk_csr_CompactColumns(gk_csr_t *mat);
METIS_API(void) gk_csr_SortIndices(gk_csr_t *mat, int what);
METIS_API(void) gk_csr_CreateIndex(gk_csr_t *mat, int what);
METIS_API(void) gk_csr_Normalize(gk_csr_t *mat, int what, int norm);
METIS_API(void) gk_csr_Scale(gk_csr_t *mat, int type);
METIS_API(void) gk_csr_ComputeSums(gk_csr_t *mat, int what);
METIS_API(void) gk_csr_ComputeSquaredNorms(gk_csr_t *mat, int what);
METIS_API(float) gk_csr_ComputeSimilarity(gk_csr_t *mat, int i1, int i2, int what, int simtype);
METIS_API(int) gk_csr_GetSimilarRows(gk_csr_t *mat, int nqterms, int *qind, float *qval,
        int simtype, int nsim, float minsim, gk_fkv_t *hits, int *_imarker,
        gk_fkv_t *i_cand);



/* itemsets.c */
METIS_API(void) gk_find_frequent_itemsets(int ntrans, ssize_t *tranptr, int *tranind,
        int minfreq, int maxfreq, int minlen, int maxlen,
        void (*process_itemset)(void *stateptr, int nitems, int *itemind,
                                int ntrans, int *tranind),
        void *stateptr);


/* evaluate.c */
METIS_API(float) ComputeAccuracy(int n, gk_fkv_t *list);
METIS_API(float) ComputeROCn(int n, int maxN, gk_fkv_t *list);
METIS_API(float) ComputeMedianRFP(int n, gk_fkv_t *list);
METIS_API(float) ComputeMean (int n, float *values);
METIS_API(float) ComputeStdDev(int  n, float *values);


/* mcore.c */
METIS_API(gk_mcore_t*) gk_mcoreCreate(size_t coresize);
METIS_API(gk_mcore_t*) gk_gkmcoreCreate();
METIS_API(void) gk_mcoreDestroy(gk_mcore_t **r_mcore, int showstats);
METIS_API(void) gk_gkmcoreDestroy(gk_mcore_t **r_mcore, int showstats);
METIS_API(void) *gk_mcoreMalloc(gk_mcore_t *mcore, size_t nbytes);
METIS_API(void) gk_mcorePush(gk_mcore_t *mcore);
METIS_API(void) gk_gkmcorePush(gk_mcore_t *mcore);
METIS_API(void) gk_mcorePop(gk_mcore_t *mcore);
METIS_API(void) gk_gkmcorePop(gk_mcore_t *mcore);
METIS_API(void) gk_mcoreAdd(gk_mcore_t *mcore, int type, size_t nbytes, void *ptr);
METIS_API(void) gk_gkmcoreAdd(gk_mcore_t *mcore, int type, size_t nbytes, void *ptr);
METIS_API(void) gk_mcoreDel(gk_mcore_t *mcore, void *ptr);
METIS_API(void) gk_gkmcoreDel(gk_mcore_t *mcore, void *ptr);

/* rw.c */
METIS_API(int) gk_rw_PageRank(gk_csr_t *mat, float lamda, float eps, int max_niter, float *pr);


/* graph.c */
METIS_API(gk_graph_t*)gk_graph_Create();
METIS_API(void) gk_graph_Init(gk_graph_t *graph);
METIS_API(void) gk_graph_Free(gk_graph_t **graph);
METIS_API(void) gk_graph_FreeContents(gk_graph_t *graph);
METIS_API(gk_graph_t*)gk_graph_Read(char *filename, int format, int isfewgts,
                    int isfvwgts, int isfvsizes);
METIS_API(void) gk_graph_Write(gk_graph_t *graph, char *filename, int format);
METIS_API(gk_graph_t*) gk_graph_Dup(gk_graph_t *graph);
METIS_API(gk_graph_t*) gk_graph_ExtractSubgraph(gk_graph_t *graph, int vstart, int nvtxs);
METIS_API(gk_graph_t*) gk_graph_Reorder(gk_graph_t *graph, int32_t *perm, int32_t *iperm);
METIS_API(int) gk_graph_FindComponents(gk_graph_t *graph, int32_t *cptr, int32_t *cind);
METIS_API(void) gk_graph_ComputeBFSOrdering(gk_graph_t *graph, int v, int32_t **r_perm, 
         int32_t **r_iperm);
METIS_API(void) gk_graph_ComputeBestFOrdering0(gk_graph_t *graph, int v, int type,
              int32_t **r_perm, int32_t **r_iperm);
METIS_API(void) gk_graph_ComputeBestFOrdering(gk_graph_t *graph, int v, int type,
              int32_t **r_perm, int32_t **r_iperm);
METIS_API(void) gk_graph_SingleSourceShortestPaths(gk_graph_t *graph, int v, void **r_sps);




#ifdef __cplusplus
}
#endif


#endif

