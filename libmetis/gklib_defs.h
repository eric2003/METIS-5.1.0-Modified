/*!
\file
\brief Data structures and prototypes for GKlib integration

\date  Started 12/23/2008
\author George
\version\verbatim $Id: gklib_defs.h 10395 2011-06-23 23:28:06Z karypis $ \endverbatim
*/

#ifndef _LIBMETIS_GKLIB_H_
#define _LIBMETIS_GKLIB_H_
#include "metis_def.h"
#include "gklib_rename.h"

/*************************************************************************/
/*! Stores a weighted edge */
/*************************************************************************/
typedef struct {
  idx_t u, v, w;               /*!< Edge (u,v) with weight w */
} uvw_t;

/*************************************************************************
* Define various data structure using GKlib's templates.
**************************************************************************/
GK_MKKEYVALUE_T(ikv_t, idx_t, idx_t)
GK_MKKEYVALUE_T(rkv_t, real_t, idx_t)
GK_MKPQUEUE_T(ipq_t, ikv_t)
GK_MKPQUEUE_T(rpq_t, rkv_t)


/* gklib.c */
GK_MKBLAS_PROTO(i, idx_t, idx_t)
GK_MKBLAS_PROTO(r, real_t, real_t)
GK_MKALLOC_PROTO(i, idx_t)
GK_MKALLOC_PROTO(r, real_t)
GK_MKALLOC_PROTO(ikv, ikv_t)
GK_MKALLOC_PROTO(rkv, rkv_t)
GK_MKPQUEUE_PROTO(ipq, ipq_t, idx_t, idx_t)
GK_MKPQUEUE_PROTO(rpq, rpq_t, real_t, idx_t)
GK_MKRANDOM_PROTO(i, idx_t, idx_t)
GK_MKARRAY2CSR_PROTO(i, idx_t)
METIS_API(void) isorti(size_t n, idx_t *base);
METIS_API(void) isortd(size_t n, idx_t *base);
METIS_API(void) rsorti(size_t n, real_t *base);
METIS_API(void) rsortd(size_t n, real_t *base);
METIS_API(void) ikvsorti(size_t n, ikv_t *base);
METIS_API(void) ikvsortii(size_t n, ikv_t *base);
METIS_API(void) ikvsortd(size_t n, ikv_t *base);
METIS_API(void) rkvsorti(size_t n, rkv_t *base);
METIS_API(void) rkvsortd(size_t n, rkv_t *base);
METIS_API(void) uvwsorti(size_t n, uvw_t *base);

#endif
