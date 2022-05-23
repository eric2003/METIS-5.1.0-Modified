/*
 * proto.h 
 *
 * This file contains function prototypes
 *
 * Started 11/1/99
 * George
 *
 * $Id: proto.h 10513 2011-07-07 22:06:03Z karypis $
 *
 */

#ifndef _PROTOBIN_H_
#define _PROTOBIN_H_
#include "metis_def.h"

/* io.c */ 
METIS_API(graph_t*) ReadGraph(params_t* params);
METIS_API(mesh_t *) ReadMesh(params_t* params);
METIS_API(void) ReadTPwgts(params_t *params, idx_t ncon);
METIS_API(void) ReadPOVector(graph_t *graph, char *filename, idx_t *vector);
METIS_API(void) WritePartition(char *, idx_t *, idx_t, idx_t);
METIS_API(void) WriteMeshPartition(char *, idx_t, idx_t, idx_t *, idx_t, idx_t *);
METIS_API(void) WritePermutation(char *, idx_t *, idx_t);
METIS_API(void) WriteGraph(graph_t *graph, char *filename);


/* smbfactor.c */
METIS_API(void) ComputeFillIn(graph_t *graph, idx_t *perm, idx_t *iperm,
         size_t *r_maxlnz, size_t *r_opc);
METIS_API(idx_t) smbfct(idx_t neqns, idx_t *xadj, idx_t *adjncy, idx_t *perm,
          idx_t *invp, idx_t *xlnz, idx_t *maxlnz, idx_t *xnzsub, 
          idx_t *nzsub, idx_t *maxsub);


/* cmdline.c */
METIS_API(params_t *) parse_cmdline(int argc, char *argv[]);

/* gpmetis.c */
METIS_API(void) GPPrintInfo(params_t *params, graph_t *graph);
METIS_API(void) GPReportResults(params_t *params, graph_t *graph, idx_t *part, idx_t edgecut);

/* ndmetis.c */
METIS_API(void) NDPrintInfo(params_t *params, graph_t *graph);
METIS_API(void) NDReportResults(params_t *params, graph_t *graph, idx_t *perm, idx_t *iperm);

/* mpmetis.c */
METIS_API(void) MPPrintInfo(params_t *params, mesh_t *mesh);
METIS_API(void) MPReportResults(params_t *params, mesh_t *mesh, idx_t *epart, idx_t *npart, 
         idx_t edgecut);

/* m2gmetis.c */
METIS_API(void) M2GPrintInfo(params_t *params, mesh_t *mesh);
METIS_API(void) M2GReportResults(params_t *params, mesh_t *mesh, graph_t *graph);

/* stat.c */
METIS_API(void) ComputePartitionInfo(params_t *params, graph_t *graph, idx_t *where);


#endif 
