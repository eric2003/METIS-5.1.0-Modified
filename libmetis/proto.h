/*
 * Copyright 1997, Regents of the University of Minnesota
 *
 * proto.h
 *
 * This file contains header files
 *
 * Started 10/19/95
 * George
 *
 * $Id: proto.h 13933 2013-03-29 22:20:46Z karypis $
 *
 */

#ifndef _LIBMETIS_PROTO_H_
#define _LIBMETIS_PROTO_H_
#include "metis_def.h"
/* auxapi.c */

/* balance.c */
METIS_API(void) Balance2Way(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
METIS_API(void) Bnd2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
METIS_API(void) General2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);
METIS_API(void) McGeneral2WayBalance(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts);


/* bucketsort.c */
METIS_API(void) BucketSortKeysInc(ctrl_t *ctrl, idx_t n, idx_t max, idx_t *keys,
         idx_t *tperm, idx_t *perm);


/* checkgraph.c */
METIS_API(int) CheckGraph(graph_t *graph, int numflag, int verbose);
METIS_API(int) CheckInputGraphWeights(idx_t nvtxs, idx_t ncon, idx_t *xadj, idx_t *adjncy,
        idx_t *vwgt, idx_t *vsize, idx_t *adjwgt);
METIS_API(graph_t*) FixGraph(graph_t *graph);


/* coarsen.c */
METIS_API(graph_t*) CoarsenGraph(ctrl_t *ctrl, graph_t *graph);
METIS_API(graph_t*) CoarsenGraphNlevels(ctrl_t *ctrl, graph_t *graph, idx_t nlevels);
METIS_API(idx_t) Match_RM(ctrl_t *ctrl, graph_t *graph);
METIS_API(idx_t) Match_SHEM(ctrl_t *ctrl, graph_t *graph);
METIS_API(idx_t) Match_2Hop(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t nunmatched);
METIS_API(idx_t) Match_2HopAny(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree);
METIS_API(idx_t) Match_2HopAll(ctrl_t *ctrl, graph_t *graph, idx_t *perm, idx_t *match,
          idx_t cnvtxs, size_t *r_nunmatched, size_t maxdegree);
METIS_API(void) PrintCGraphStats(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) CreateCoarseGraph(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match);
METIS_API(void) CreateCoarseGraphNoMask(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match);
METIS_API(void) CreateCoarseGraphPerm(ctrl_t *ctrl, graph_t *graph, idx_t cnvtxs, 
         idx_t *match, idx_t *perm);
METIS_API(graph_t*) SetupCoarseGraph(graph_t *graph, idx_t cnvtxs, idx_t dovsize);
METIS_API(void) ReAdjustMemory(ctrl_t *ctrl, graph_t *graph, graph_t *cgraph);



/* compress.c */
METIS_API(graph_t *) CompressGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t *xadj, idx_t *adjncy,
             idx_t *vwgt, idx_t *cptr, idx_t *cind);
METIS_API(graph_t*) PruneGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t *xadj, idx_t *adjncy,
             idx_t *vwgt, idx_t *iperm, real_t factor);


/* contig.c */
METIS_API(idx_t) FindPartitionInducedComponents(graph_t *graph, idx_t *where, 
          idx_t *cptr, idx_t *cind);
METIS_API(void) ComputeBFSOrdering(ctrl_t *ctrl, graph_t *graph, idx_t *bfsperm);
METIS_API(idx_t) IsConnected(graph_t *graph, idx_t report);
METIS_API(idx_t) IsConnectedSubdomain(ctrl_t *, graph_t *, idx_t, idx_t);
METIS_API(idx_t) FindSepInducedComponents(ctrl_t *, graph_t *, idx_t *, idx_t *);
METIS_API(void) EliminateComponents(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) MoveGroupContigForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid, 
         idx_t *ptr, idx_t *ind);
METIS_API(void) MoveGroupContigForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t gid,
         idx_t *ptr, idx_t *ind, idx_t *vmarker, idx_t *pmarker,
         idx_t *modind);


/* debug.c */
METIS_API(idx_t) ComputeCut(graph_t *graph, idx_t *where);
METIS_API(idx_t) ComputeVolume(graph_t *, idx_t *);
METIS_API(idx_t) ComputeMaxCut(graph_t *graph, idx_t nparts, idx_t *where);
METIS_API(idx_t) CheckBnd(graph_t *);
METIS_API(idx_t) CheckBnd2(graph_t *);
METIS_API(idx_t) CheckNodeBnd(graph_t *, idx_t);
METIS_API(idx_t) CheckRInfo(ctrl_t *ctrl, ckrinfo_t *rinfo);
METIS_API(idx_t) CheckNodePartitionParams(graph_t *);
METIS_API(idx_t) IsSeparable(graph_t *);
METIS_API(void) CheckKWayVolPartitionParams(ctrl_t *ctrl, graph_t *graph);


/* fm.c */
METIS_API(void) FM_2WayRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
METIS_API(void) FM_2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
METIS_API(void) FM_Mc2WayCutRefine(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niter);
METIS_API(void) SelectQueue(graph_t *graph, real_t *pijbm, real_t *ubfactors, rpq_t **queues, 
         idx_t *from, idx_t *cnum);
METIS_API(void) Print2WayRefineStats(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, 
         real_t deltabal, idx_t mincutorder);


/* fortran.c */
METIS_API(void) Change2CNumbering(idx_t, idx_t *, idx_t *);
METIS_API(void) Change2FNumbering(idx_t, idx_t *, idx_t *, idx_t *);
METIS_API(void) Change2FNumbering2(idx_t, idx_t *, idx_t *);
METIS_API(void) Change2FNumberingOrder(idx_t, idx_t *, idx_t *, idx_t *, idx_t *);
METIS_API(void) ChangeMesh2CNumbering(idx_t n, idx_t *ptr, idx_t *ind);
METIS_API(void) ChangeMesh2FNumbering(idx_t n, idx_t *ptr, idx_t *ind, idx_t nvtxs,
         idx_t *xadj, idx_t *adjncy);
METIS_API(void) ChangeMesh2FNumbering2(idx_t ne, idx_t nn, idx_t *ptr, idx_t *ind,
         idx_t *epart, idx_t *npart);


/* graph.c */
METIS_API(graph_t*) SetupGraph(ctrl_t *ctrl, idx_t nvtxs, idx_t ncon, idx_t *xadj,
             idx_t *adjncy, idx_t *vwgt, idx_t *vsize, idx_t *adjwgt);
METIS_API(void) SetupGraph_tvwgt(graph_t *graph);
METIS_API(void) SetupGraph_label(graph_t *graph);
METIS_API(graph_t*) SetupSplitGraph(graph_t *graph, idx_t snvtxs, idx_t snedges);
METIS_API(graph_t*) CreateGraph(void);
METIS_API(void) InitGraph(graph_t *graph);
METIS_API(void) FreeRData(graph_t *graph);
METIS_API(void) FreeGraph(graph_t **graph);


/* initpart.c */
METIS_API(void) Init2WayPartition(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) InitSeparator(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
METIS_API(void) RandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) GrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) McRandomBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) McGrowBisection(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) GrowBisectionNode(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);
METIS_API(void) GrowBisectionNode2(ctrl_t *ctrl, graph_t *graph, real_t *ntpwgts, idx_t niparts);


/* kmetis.c */
METIS_API(idx_t) MlevelKWayPartitioning(ctrl_t *ctrl, graph_t *graph, idx_t *part);
METIS_API(void) InitKWayPartitioning(ctrl_t *ctrl, graph_t *graph);


/* kwayfm.c */
METIS_API(void) Greedy_KWayOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
METIS_API(void) Greedy_KWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
METIS_API(void) Greedy_KWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
METIS_API(void) Greedy_McKWayCutOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
METIS_API(void) Greedy_McKWayVolOptimize(ctrl_t *ctrl, graph_t *graph, idx_t niter, 
         real_t ffactor, idx_t omode);
METIS_API(idx_t) IsArticulationNode(idx_t i, idx_t *xadj, idx_t *adjncy, idx_t *where,
          idx_t *bfslvl, idx_t *bfsind, idx_t *bfsmrk);
METIS_API(void) KWayVolUpdate(ctrl_t *ctrl, graph_t *graph, idx_t v, idx_t from,
         idx_t to, ipq_t *queue, idx_t *vstatus, idx_t *r_nupd, idx_t *updptr,
         idx_t *updind, idx_t bndtype, idx_t *vmarker, idx_t *pmarker,
         idx_t *modind);


/* kwayrefine.c */
METIS_API(void) RefineKWay(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph);
METIS_API(void) AllocateKWayPartitionMemory(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) ComputeKWayPartitionParams(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) ProjectKWayPartition(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) ComputeKWayBoundary(ctrl_t *ctrl, graph_t *graph, idx_t bndtype);
METIS_API(void) ComputeKWayVolGains(ctrl_t *ctrl, graph_t *graph);
METIS_API(int) IsBalanced(ctrl_t *ctrl, graph_t *graph, real_t ffactor);


/* mcutil.c */
METIS_API(int) rvecle(idx_t n, real_t *x, real_t *y);
METIS_API(int) rvecge(idx_t n, real_t *x, real_t *y);
METIS_API(int) rvecsumle(idx_t n, real_t *x1, real_t *x2, real_t *y);
real_t rvecmaxdiff(idx_t n, real_t *x, real_t *y);
METIS_API(int) ivecle(idx_t n, idx_t *x, idx_t *z);
METIS_API(int) ivecge(idx_t n, idx_t *x, idx_t *z);
METIS_API(int) ivecaxpylez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z);
METIS_API(int) ivecaxpygez(idx_t n, idx_t a, idx_t *x, idx_t *y, idx_t *z);
METIS_API(int) BetterVBalance(idx_t ncon, real_t *itvwgt, idx_t *v_vwgt, idx_t *u1_vwgt,
            idx_t *u2_vwgt);
METIS_API(int) BetterBalance2Way(idx_t n, real_t *x, real_t *y);
METIS_API(int) BetterBalanceKWay(idx_t ncon, idx_t *vwgt, real_t *itvwgt, idx_t a1,
        idx_t *pt1, real_t *bm1, idx_t a2, idx_t *pt2, real_t *bm2);
METIS_API(real_t) ComputeLoadImbalance(graph_t *graph, idx_t nparts, real_t *pijbm);
METIS_API(real_t) ComputeLoadImbalanceDiff(graph_t *graph, idx_t nparts, real_t *pijbm,
           real_t *ubvec);
METIS_API(real_t)  ComputeLoadImbalanceDiffVec(graph_t *graph, idx_t nparts, real_t *pijbm,
         real_t *ubfactors, real_t *diffvec);
METIS_API(void) ComputeLoadImbalanceVec(graph_t *graph, idx_t nparts, real_t *pijbm,
             real_t *lbvec);


/* mesh.c */
METIS_API(void) CreateGraphDual(idx_t ne, idx_t nn, idx_t *eptr, idx_t *eind, idx_t ncommon,
          idx_t **r_xadj, idx_t **r_adjncy);
METIS_API(idx_t) FindCommonElements(idx_t qid, idx_t elen, idx_t *eind, idx_t *nptr,
          idx_t *nind, idx_t *eptr, idx_t ncommon, idx_t *marker, idx_t *nbrs);
METIS_API(void) CreateGraphNodal(idx_t ne, idx_t nn, idx_t *eptr, idx_t *eind, idx_t **r_xadj, 
          idx_t **r_adjncy);
METIS_API(idx_t) FindCommonNodes(idx_t qid, idx_t nelmnts, idx_t *elmntids, idx_t *eptr,
          idx_t *eind, idx_t *marker, idx_t *nbrs);
METIS_API(mesh_t*) CreateMesh(void);
METIS_API(void) InitMesh(mesh_t *mesh);  
METIS_API(void) FreeMesh(mesh_t **mesh);


/* meshpart.c */
METIS_API(void) InduceRowPartFromColumnPart(idx_t nrows, idx_t *rowptr, idx_t *rowind,
         idx_t *rpart, idx_t *cpart, idx_t nparts, real_t *tpwgts);


/* minconn.c */
METIS_API(void) ComputeSubDomainGraph(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) UpdateEdgeSubDomainGraph(ctrl_t *ctrl, idx_t u, idx_t v, idx_t ewgt, 
         idx_t *r_maxndoms);
METIS_API(void) PrintSubDomainGraph(graph_t *graph, idx_t nparts, idx_t *where);
METIS_API(void) EliminateSubDomainEdges(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) MoveGroupMinConnForCut(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind);
METIS_API(void) MoveGroupMinConnForVol(ctrl_t *ctrl, graph_t *graph, idx_t to, idx_t nind, 
         idx_t *ind, idx_t *vmarker, idx_t *pmarker, idx_t *modind);


/* mincover.o */
METIS_API(void) MinCover(idx_t *, idx_t *, idx_t, idx_t, idx_t *, idx_t *);
METIS_API(idx_t) MinCover_Augment(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t *, idx_t);
METIS_API(void) MinCover_Decompose(idx_t *, idx_t *, idx_t, idx_t, idx_t *, idx_t *, idx_t *);
METIS_API(void) MinCover_ColDFS(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t);
METIS_API(void) MinCover_RowDFS(idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t);


/* mmd.c */
METIS_API(void) genmmd(idx_t, idx_t *, idx_t *, idx_t *, idx_t *, idx_t , idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t *);
METIS_API(void) mmdelm(idx_t, idx_t *xadj, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t);
METIS_API(idx_t) mmdint(idx_t, idx_t *xadj, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *);
METIS_API(void) mmdnum(idx_t, idx_t *, idx_t *, idx_t *);
METIS_API(void) mmdupd(idx_t, idx_t, idx_t *, idx_t *, idx_t, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t *, idx_t, idx_t *tag);


/* ometis.c */
METIS_API(void) MlevelNestedDissection(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx);
METIS_API(void) MlevelNestedDissectionCC(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx);
METIS_API(void) MlevelNodeBisectionMultiple(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) MlevelNodeBisectionL2(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
METIS_API(void) MlevelNodeBisectionL1(ctrl_t *ctrl, graph_t *graph, idx_t niparts);
METIS_API(void) SplitGraphOrder(ctrl_t *ctrl, graph_t *graph, graph_t **r_lgraph, 
         graph_t **r_rgraph);
METIS_API(graph_t**) SplitGraphOrderCC(ctrl_t *ctrl, graph_t *graph, idx_t ncmps,
              idx_t *cptr, idx_t *cind);
METIS_API(void) MMDOrder(ctrl_t *ctrl, graph_t *graph, idx_t *order, idx_t lastvtx);


/* options.c */
METIS_API(ctrl_t*) SetupCtrl(moptype_et optype, idx_t *options, idx_t ncon, idx_t nparts,
            real_t *tpwgts, real_t *ubvec);
METIS_API(void) SetupKWayBalMultipliers(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) Setup2WayBalMultipliers(ctrl_t *ctrl, graph_t *graph, real_t *tpwgts);
METIS_API(void) PrintCtrl(ctrl_t *ctrl);
METIS_API(int) CheckParams(ctrl_t *ctrl);
METIS_API(void) FreeCtrl(ctrl_t **r_ctrl);


/* parmetis.c */
METIS_API(void) MlevelNestedDissectionP(ctrl_t *ctrl, graph_t *graph, idx_t *order,
         idx_t lastvtx, idx_t npes, idx_t cpos, idx_t *sizes);
METIS_API(void) FM_2WayNodeRefine1SidedP(ctrl_t *ctrl, graph_t *graph, idx_t *hmarker, 
         real_t ubfactor, idx_t npasses);
METIS_API(void) FM_2WayNodeRefine2SidedP(ctrl_t *ctrl, graph_t *graph, idx_t *hmarker, 
         real_t ubfactor, idx_t npasses);


/* pmetis.c */
METIS_API(idx_t) MlevelRecursiveBisection(ctrl_t *ctrl, graph_t *graph, idx_t nparts, 
          idx_t *part, real_t *tpwgts, idx_t fpart);
METIS_API(idx_t) MultilevelBisect(ctrl_t *ctrl, graph_t *graph, real_t *tpwgts);
METIS_API(void) SplitGraphPart(ctrl_t *ctrl, graph_t *graph, graph_t **r_lgraph, graph_t **r_rgraph);


/* refine.c */
METIS_API(void) Refine2Way(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph, real_t *rtpwgts);
METIS_API(void) Allocate2WayPartitionMemory(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) Compute2WayPartitionParams(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) Project2WayPartition(ctrl_t *ctrl, graph_t *graph);


/* separator.c */
METIS_API(void) ConstructSeparator(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) ConstructMinCoverSeparator(ctrl_t *ctrl, graph_t *graph);


/* sfm.c */
METIS_API(void) FM_2WayNodeRefine2Sided(ctrl_t *ctrl, graph_t *graph, idx_t niter);
METIS_API(void) FM_2WayNodeRefine1Sided(ctrl_t *ctrl, graph_t *graph, idx_t niter);
METIS_API(void) FM_2WayNodeBalance(ctrl_t *ctrl, graph_t *graph);


/* srefine.c */
METIS_API(void) Refine2WayNode(ctrl_t *ctrl, graph_t *orggraph, graph_t *graph);
METIS_API(void) Allocate2WayNodePartitionMemory(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) Compute2WayNodePartitionParams(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) Project2WayNodePartition(ctrl_t *ctrl, graph_t *graph);


/* stat.c */
METIS_API(void) ComputePartitionInfoBipartite(graph_t *, idx_t, idx_t *);
METIS_API(void) ComputePartitionBalance(graph_t *, idx_t, idx_t *, real_t *);
METIS_API(real_t) ComputeElementBalance(idx_t, idx_t, idx_t *);


/* timing.c */
METIS_API(void) InitTimers(ctrl_t *);
METIS_API(void) PrintTimers(ctrl_t *);

/* util.c */
METIS_API(idx_t) iargmax_strd(size_t, idx_t *, idx_t);
METIS_API(idx_t) iargmax_nrm(size_t n, idx_t *x, real_t *y);
METIS_API(idx_t) iargmax2_nrm(size_t n, idx_t *x, real_t *y);
METIS_API(idx_t) rargmax2(size_t, real_t *);
METIS_API(void) InitRandom(idx_t);
METIS_API(int) metis_rcode(int sigrval);



/* wspace.c */
METIS_API(void) AllocateWorkSpace(ctrl_t *ctrl, graph_t *graph);
METIS_API(void) AllocateRefinementWorkSpace(ctrl_t *ctrl, idx_t nbrpoolsize);
METIS_API(void) FreeWorkSpace(ctrl_t *ctrl);
METIS_API(void) *wspacemalloc(ctrl_t *ctrl, size_t nbytes);
METIS_API(void) wspacepush(ctrl_t *ctrl);
METIS_API(void) wspacepop(ctrl_t *ctrl);
METIS_API(idx_t *) iwspacemalloc(ctrl_t *, idx_t);
METIS_API(real_t*) rwspacemalloc(ctrl_t *, idx_t);
METIS_API(ikv_t*) ikvwspacemalloc(ctrl_t *, idx_t);
METIS_API(void) cnbrpoolReset(ctrl_t *ctrl);
METIS_API(idx_t) cnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs);
METIS_API(void) vnbrpoolReset(ctrl_t *ctrl);
METIS_API(idx_t) vnbrpoolGetNext(ctrl_t *ctrl, idx_t nnbrs);


#endif
