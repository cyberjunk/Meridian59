// Meridian 59, Copyright 1994-2012 Andrew Kirmse and Chris Kirmse.
// All rights reserved.
//
// This software is distributed under a license that is described in
// the LICENSE file that accompanies it.
//
// Meridian is a registered trademark.
/*
* astar.h:
*/

#ifndef _ASTAR_H
#define _ASTAR_H

#define EDGESCACHEENABLED 1

#define CLOSEENOUGHDIST   3
#define DESTBLOCKIGNORE   3
#define ASTARENABLED      1
#define NUMNEIGHBOURS     8

#define LCHILD(x) (2 * x + 1)
#define RCHILD(x) (2 * x + 2)
#define PARENT(x) ((x-1) / 2)

#define COST       1.0f
#define COST_DIAG  ((float)M_SQRT2)

#define EDGECACHE_KNOWS_N  0x0001
#define EDGECACHE_KNOWS_NE 0x0002
#define EDGECACHE_KNOWS_E  0x0004
#define EDGECACHE_KNOWS_SE 0x0008
#define EDGECACHE_KNOWS_S  0x0010
#define EDGECACHE_KNOWS_SW 0x0020
#define EDGECACHE_KNOWS_W  0x0040
#define EDGECACHE_KNOWS_NW 0x0080
#define EDGECACHE_CAN_N    0x0101
#define EDGECACHE_CAN_NE   0x0202
#define EDGECACHE_CAN_E    0x0404
#define EDGECACHE_CAN_SE   0x0808
#define EDGECACHE_CAN_S    0x1010
#define EDGECACHE_CAN_SW   0x2020
#define EDGECACHE_CAN_W    0x4040
#define EDGECACHE_CAN_NW   0x8080

typedef struct room_type room_type;
typedef struct BspLeaf BspLeaf;
typedef struct astar_node_data astar_node_data;
typedef struct astar_node astar_node;

typedef struct astar_node_data
{
   float       cost;
   float       heuristic;
   float       combined;
   astar_node* parent;
   astar_node* heapslot;
   int         heapindex;
   bool        isInClosedList;
   bool        isBlocked;
} astar_node_data;

typedef struct astar_node
{
   int              Row;
   int              Col;
   V2               Location;
   BspLeaf*         Leaf;
   astar_node_data* Data;
   astar_node*      Neighbours[NUMNEIGHBOURS];
   unsigned short*  Edges;
} astar_node;

typedef struct astar
{
   astar_node_data* NodesData;
   int              NodesDataSize;
   unsigned short*  EdgesCache;
   int              EdgesCacheSize;
   astar_node**     Grid;
   astar_node*      EndNode;
   astar_node*      LastNode;
   int              ObjectID;
   int              HeapSize;
} astar;

void AStarGenerateGrid(room_type* Room);
void AStarFreeGrid(room_type* Room);
void AStarClearEdgesCache(room_type* Room);
bool AStarGetStepTowards(room_type* Room, V2* S, V2* E, V2* P, unsigned int* Flags, int ObjectID);

#endif /*#ifndef _ASTAR_H */