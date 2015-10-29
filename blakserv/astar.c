// Meridian 59, Copyright 1994-2012 Andrew Kirmse and Chris Kirmse.
// All rights reserved.
//
// This software is distributed under a license that is described in
// the LICENSE file that accompanies it.
//
// Meridian is a registered trademark.
/*
* astar.c
*

Improvements over the old implementation:
  * Allocation of gridmemory is only done once when room is loaded
  * Node information is split up between persistent data and data
    which needs to be cleared for each algorithm and can be by single ZeroMemory
  * Whether a node is in the closed set is not saved and looked up by an additional list,
    but rather simply saved by a flag on that node.
  * Uses a binary heap to store the open set
*/

#include "blakserv.h"

astar AStar;

#pragma region HEAP
// refers to the i-th element on the heap
#define HEAP(a, i) ((a)->NodesData[i].heapslot)

__inline bool AStarHeapCheck(astar* Astar, room_type* Room, int i)
{
   int squares = Room->rowshighres * Room->colshighres;

   if (i >= squares)
      return true;

   if (!HEAP(Astar, i))
      return true;

   float our = HEAP(Astar, i)->Data->combined;
   int lchild = LCHILD(i);
   int rchild = RCHILD(i);

   if (lchild < squares)
   {
      astar_node* node = HEAP(Astar, lchild);
      if (node)
      {
         if (node->Data->combined < our)
            return false;
      }
   }
   if (rchild < squares)
   {
      astar_node* node = HEAP(Astar, rchild);
      if (node)
      {
         if (node->Data->combined < our)
            return false;
      }
   }

   bool retval = AStarHeapCheck(Astar, Room, lchild);
   if (!retval)
      return false;

   return AStarHeapCheck(Astar, Room, rchild);
}

__inline void AStarWriteHeapToFile(astar* Astar, room_type* Room)
{
   int rows = Room->rowshighres;
   int cols = Room->colshighres;
   char* rowstring = (char *)AllocateMemory(MALLOC_ID_ROOM, 60000);
   FILE *fp = fopen("heapdebug.txt", "a");
   if (fp)
   {
      int maxlayer = 9;	
      int treesize = cols * rows;
      int rangestart = 0;
      int rangesize = 1;
      for (int i = 1; i < maxlayer; i++)
      {
         if (treesize < rangestart + rangesize)
            break;

         sprintf(rowstring, "L:%.2i", i);

         for (int k = 0; k < rangesize; k++)
         {				
            astar_node* node = Astar->NodesData[rangestart + k].heapslot;

            if (node)
               sprintf(rowstring, "%s|%6.2f|", rowstring, node->Data->combined);
            else
               sprintf(rowstring, "%s|XXXXXX|", rowstring);
         }

         sprintf(rowstring, "%s\n", rowstring);
         fputs(rowstring, fp);
			
         rangestart = rangestart + rangesize; 
         rangesize *= 2;		
      }
      sprintf(rowstring, "%s\n", rowstring);
      fclose(fp);
   }
   FreeMemory(MALLOC_ID_ROOM, rowstring, 60000);
}

__inline void AStarHeapSwap(astar* Astar, int idx1, int idx2)
{
   astar_node* node1 = HEAP(Astar, idx1);
   astar_node* node2 = HEAP(Astar, idx2);	
   HEAP(Astar, idx1) = node2;
   HEAP(Astar, idx2) = node1;
   node1->Data->heapindex = idx2;
   node2->Data->heapindex = idx1;
}

__inline void AStarHeapMoveUp(astar* Astar, int Index)
{
   int i = Index;
   while (i > 0 && HEAP(Astar, i)->Data->combined < HEAP(Astar, PARENT(i))->Data->combined)
   {
      AStarHeapSwap(Astar, i, PARENT(i));
      i = PARENT(i);
   }
}

__inline void AStarHeapHeapify(astar* Astar, int Index)
{
   int i = Index;
   do 
   {
      int min = i;
      if (HEAP(Astar, LCHILD(i)) && HEAP(Astar, LCHILD(i))->Data->combined < HEAP(Astar, min)->Data->combined)
         min = LCHILD(i);
      if (HEAP(Astar, RCHILD(i)) && HEAP(Astar, RCHILD(i))->Data->combined < HEAP(Astar, min)->Data->combined)
         min = RCHILD(i);
      if (min == i)
         break;
      AStarHeapSwap(Astar, i, min);
      i = min;
   } 
   while (true);
}

__inline void AStarHeapInsert(astar* Astar, astar_node* Node)
{
   // save index of this node
   Node->Data->heapindex = Astar->HeapSize;

   // add node at the end
   HEAP(Astar, Node->Data->heapindex) = Node;

   // increment
   Astar->HeapSize++;

   // push node up until in place
   AStarHeapMoveUp(Astar, Node->Data->heapindex);
}

__inline void AStarHeapRemoveFirst(astar* Astar)
{
   int lastIdx = Astar->HeapSize - 1;

   // no elements
   if (lastIdx < 0)
      return;

   // decrement size
   Astar->HeapSize--;

   // more than 1
   if (lastIdx > 0)
   {
      // put last at root
      AStarHeapSwap(Astar, 0, lastIdx);

      // zero out the previous root at swapped slot
      HEAP(Astar, lastIdx)->Data->heapindex = 0;
      HEAP(Astar, lastIdx) = NULL;

      // reorder tree
      AStarHeapHeapify(Astar, 0);
   }

   // only one, clear head
   else
   {
      HEAP(Astar, 0)->Data->heapindex = 0;
      HEAP(Astar, 0) = NULL;
   }
}
#pragma endregion

#pragma region ASTAR 
__inline void AStarAddBlockers(astar* Astar)
{
   Blocker* b;
	
   b = Astar->Room->Blocker;
   while (b)
   {
      // Don't add self.
      if (b->ObjectID == Astar->ObjectID)
      {
         b = b->Next;
         continue;
      }

      // Get blocker coords
      int row = (int)roundf(b->Position.Y / 256.0f);
      int col = (int)roundf(b->Position.X / 256.0f);
	  
      // Don't add blockers at the target coords.
      if (abs(row - Astar->EndNode->Meta->Row) < DESTBLOCKIGNORE &&
          abs(col - Astar->EndNode->Meta->Col) < DESTBLOCKIGNORE)
      {
         b = b->Next;
         continue;
      }

	  //todo: use neighbour[] ptrs here?
      // Mark these nodes in A* grid blocked (our coord and +2 highres each dir)
      for (int rowoffset = -2; rowoffset < 3; rowoffset++)
      {
         for (int coloffset = -2; coloffset < 3; coloffset++)
         {
            int r = row + rowoffset;
            int c = col + coloffset;

            // outside
            if (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres)
               continue;

            astar_node* node = &Astar->Room->Grid[r][c];
            node->Data->isBlocked = true;
         }
      }
      b = b->Next;
   }
}

__inline bool AStarCanWalkEdge(astar* Astar, astar_node* Node, astar_node* Neighbour, unsigned short KnowsVal, unsigned short CanVal)
{
   Wall* blockWall;

#if EDGESCACHEENABLED
   bool knows = (((*Node->Edges) & KnowsVal) == KnowsVal);
   bool can;

   // not yet cached
   if (!knows)
   {
      // bsp query
      can = BSPCanMoveInRoom(Astar->Room, &Node->Meta->Location, &Neighbour->Meta->Location, Astar->ObjectID, false, &blockWall, true);

      // save to cache
      *Node->Edges |= (can) ? CanVal : KnowsVal;
   }

   // retrieve answer from cache
   else
      can = (((*Node->Edges) & CanVal) == CanVal);

   return can;

#else
   return BSPCanMoveInRoom(Astar->Room, &Node->Meta->Location, &Neighbour->Meta->Location, Astar->ObjectID, false, &blockWall, true);
#endif
}

__inline void AStarProcessNeighbour(astar* Astar, astar_node* Node, astar_node* Neighbour, float StepCost, unsigned short KnowsVal, unsigned short CanVal)
{
   // skip any neighbour outside of grid or sector, already processed or blocked
   if (!Neighbour || !Neighbour->Leaf || Neighbour->Data->isInClosedList || Neighbour->Data->isBlocked)
      return;

   // can't walk edge from node to neighbour
   if (!AStarCanWalkEdge(Astar, Node, Neighbour, KnowsVal, CanVal))
      return;

   // heuristic (~ estimated distance from node to end)
   // need to do this only once
   if (Neighbour->Data->heuristic == 0.0f)
   {
      float dx = (float)abs(Neighbour->Meta->Col - Astar->EndNode->Meta->Col);
      float dy = (float)abs(Neighbour->Meta->Row - Astar->EndNode->Meta->Row);

      // octile-distance
      Neighbour->Data->heuristic =
         COST * (dx + dy) + (COST_DIAG - 2.0f * COST) * fminf(dx, dy);

      // tie breaker and fixes h(nondiagonal) not lower exact cost
      Neighbour->Data->heuristic *= 0.999f;
   }

   // CASE 1)
   // if this candidate has no parent yet (never visited before)
   if (!Neighbour->Data->parent)
   {
      // cost to candidate is cost to Node + one step
      Neighbour->Data->parent = Node;
      Neighbour->Data->cost = Node->Data->cost + StepCost;
      Neighbour->Data->combined = Neighbour->Data->cost + Neighbour->Data->heuristic;

      // add it sorted to the open list
      AStarHeapInsert(Astar, Neighbour);
   }

   // CASE 2)
   // we already got a path to this candidate
   else
   {
      // our cost to the candidate
      float newcost = Node->Data->cost + StepCost;

      // we're cheaper, so update the candidate
      // the real cost matters here, not including the heuristic
      if (newcost < Neighbour->Data->cost)
      {
         Neighbour->Data->parent = Node;
         Neighbour->Data->cost = newcost;
         Neighbour->Data->combined = newcost + Neighbour->Data->heuristic;

         // reorder it upwards in the heap tree, don't care about downordering
         // since costs are lower and heuristic is always the same,
         // it's guaranteed to be moved up
         AStarHeapMoveUp(Astar, Neighbour->Data->heapindex);
      }
   }
}

__inline bool AStarProcessFirst(astar* Astar)
{
   // shortcuts
   astar_node* node = HEAP(Astar, 0);
   astar_node* endNode = Astar->EndNode;

   // openlist empty, unreachable
   if (!node)
      return false;

   // close enough, we're done, path found
   if (abs(node->Meta->Col - endNode->Meta->Col) < CLOSEENOUGHDIST &&
       abs(node->Meta->Row - endNode->Meta->Row) < CLOSEENOUGHDIST)
   {
      Astar->LastNode = node;
      return false;
   }

   /****************************************************************/

   int r, c;
   astar_node* n;

   // straight neighbours

   // n
   r = node->Meta->Row - 1;
   c = node->Meta->Col + 0;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST, EDGECACHE_KNOWS_N, EDGECACHE_CAN_N);
   
   // e
   r = node->Meta->Row + 0;
   c = node->Meta->Col + 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST, EDGECACHE_KNOWS_E, EDGECACHE_CAN_E);
   
   // s
   r = node->Meta->Row + 1;
   c = node->Meta->Col + 0;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST, EDGECACHE_KNOWS_S, EDGECACHE_CAN_S);

   // w
   r = node->Meta->Row + 0;
   c = node->Meta->Col - 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST, EDGECACHE_KNOWS_W, EDGECACHE_CAN_W);

   // diagonal neighbours

   // ne
   r = node->Meta->Row - 1;
   c = node->Meta->Col + 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST_DIAG, EDGECACHE_KNOWS_NE, EDGECACHE_CAN_NE);

   // se
   r = node->Meta->Row + 1;
   c = node->Meta->Col + 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST_DIAG, EDGECACHE_KNOWS_SE, EDGECACHE_CAN_SE);

   // sw
   r = node->Meta->Row + 1;
   c = node->Meta->Col - 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST_DIAG, EDGECACHE_KNOWS_SW, EDGECACHE_CAN_SW);

   // nw
   r = node->Meta->Row - 1;
   c = node->Meta->Col - 1;
   n = (r < 0 || c < 0 || r >= Astar->Room->rowshighres || c >= Astar->Room->colshighres) ? NULL : &Astar->Room->Grid[r][c];
   AStarProcessNeighbour(Astar, node, n, COST_DIAG, EDGECACHE_KNOWS_NW, EDGECACHE_CAN_NW);

   /****************************************************************/

   // mark this node processed
   node->Data->isInClosedList = true;

   // remove it from the open list
   AStarHeapRemoveFirst(Astar);

   return true;
}

void AStarWriteGridToFile(room_type* Room)
{
   int rows = Room->rowshighres;
   int cols = Room->colshighres;

   char *rowstring = (char *)AllocateMemory(MALLOC_ID_ROOM, 50000);

   FILE *fp = fopen("griddebug.txt", "w");
   if (fp)
   {
      for (int row = 0; row < rows; row++)
      {
         sprintf(rowstring, "Row %3i- ", row);
         for (int col = 0; col < cols; col++)
         {
            sprintf(rowstring, "%s|%7.3f|", rowstring, Room->Grid[row][col].Data->combined);
         }
         sprintf(rowstring, "%s \n", rowstring);
         fputs(rowstring, fp);
      }
      fclose(fp);
   }

   FreeMemory(MALLOC_ID_ROOM, rowstring, 50000);
}

void AStarGenerateGrid(room_type* Room)
{
   if (Room->colshighres > MAXGRIDCOLS || Room->rowshighres > MAXGRIDROWS)  
      eprintf("ALERT!! You tried to load a room with a grid-size beyond allowed limits! AStar will fail.");

   /**********************************************************************/

#if EDGESCACHEENABLED
   // allocate memory to cache edges (BSP queries)
   Room->EdgesCacheSize = Room->colshighres * Room->rowshighres * sizeof(unsigned short);

   Room->EdgesCache = (unsigned short*)AllocateMemory(
	   MALLOC_ID_ASTAR, Room->EdgesCacheSize);

   AStarClearEdgesCache(Room);
#endif
   /**********************************************************************/
#if PATHCACHEENABLED
   // setup path cache
   for (int i = 0; i < PATHCACHESIZE; i++)  
      Room->Paths[i] = new astar_path();
#endif
   /**********************************************************************/

   // allocate memory for the persistent nodesinfo (typical 2d array by **)
   Room->Grid = (astar_node**)AllocateMemory(
      MALLOC_ID_ASTAR, Room->rowshighres * sizeof(astar_node*));

   // allocate rows
   for (int i = 0; i < Room->rowshighres; i++)
      Room->Grid[i] = (astar_node*)AllocateMemory(
	     MALLOC_ID_ASTAR, Room->colshighres * sizeof(astar_node));

   /**********************************************************************/

   // setup all squares
   for (int i = 0; i < Room->rowshighres; i++)
   {
	  int row = (i < MAXGRIDROWS) ? i : 0;

	  for (int j = 0; j < Room->colshighres; j++)
	  {
         int col = (j < MAXGRIDCOLS) ? j : 0;	
         int idx1 = row*Room->colshighres + col;
         int idx2 = i*Room->colshighres + j;

		 astar_node* node = &Room->Grid[i][j];

         // setup reference to data (costs etc.) in erasable mem area
         // todo: change away from AStar to param
         node->Data = &AStar.NodesData[idx1];
		 node->Meta = &AStar.Grid[row][col];

#if EDGESCACHEENABLED
         // setup reference to edgesdata in edgescache
         node->Edges = &Room->EdgesCache[idx2];
#endif

		 float f1, f2, f3;

		 // mark if this square is inside or outside of room
		 BSPGetHeight(Room, &node->Meta->Location, &f1, &f2, &f3, &node->Leaf);
      }
   }
}

void AStarFreeGrid(room_type* Room)
{
#if EDGESCACHEENABLED
   // free edgescache mem
   FreeMemory(MALLOC_ID_ASTAR, Room->EdgesCache, Room->EdgesCacheSize);
#endif

#if PATHCACHEENABLED
   // free pathes list allocations
   for (int i = 0; i < PATHCACHESIZE; i++)
      delete Room->Paths[i];
#endif

   // free each row mem
   for (int i = 0; i < Room->rowshighres; i++)
      FreeMemory(MALLOC_ID_ASTAR, Room->Grid[i], Room->colshighres * sizeof(astar_node));

   // free rowsmem
   FreeMemory(MALLOC_ID_ASTAR, Room->Grid, Room->rowshighres * sizeof(astar_node*));
}

#if EDGESCACHEENABLED
void AStarClearEdgesCache(room_type* Room)
{
   ZeroMemory(Room->EdgesCache, Room->EdgesCacheSize);
}
#endif

#if PATHCACHEENABLED
void AStarClearPathCache(room_type* Room)
{
   for (int i = 0; i < PATHCACHESIZE; i++)
      Room->Paths[i]->clear();
}

bool AStarGetStepFromCache(room_type* Room, astar_node* S, astar_node* E, V2* P, unsigned int* Flags, int ObjectID)
{
   Wall* blockWall;

   for (int i = 0; i < PATHCACHESIZE; i++)
   {
      astar_path* path = Room->Paths[i];

      // a valid path would have two entries
      if (path->size() < 2)
         continue;

      // check start (better match exactly, or we might stepping back?)
      astar_node* first = path->front();
      if (first->Meta->Row != S->Meta->Row || first->Meta->Col != S->Meta->Col)
         continue;

      // check end (has small tolerance)
      astar_node* last = path->back();
      if (abs(last->Meta->Row - E->Meta->Row) > PATHCACHETOLERANCE ||
          abs(last->Meta->Col - E->Meta->Col) > PATHCACHETOLERANCE)
          continue;

      // match! now remove first, so we also match on next step
      path->pop_front();

      // get the next step endpoint
      astar_node* next = path->front();

      // make sure we can still move to this next endpoint (cares for moved objects!)
      // note: if objects block a cached path, the path will still be walked until the block occurs!
      // to improve this revalidate the whole path from the first to the last node here
	  if (!BSPCanMoveInRoom(Room, &first->Meta->Location, &next->Meta->Location, ObjectID, false, &blockWall, false))
         continue;

      // for diagonal moves mark to be long step (required for timer elapse)
      if (abs(first->Meta->Col - next->Meta->Col) &&
          abs(first->Meta->Row - next->Meta->Row))
      {
         *Flags |= ESTATE_LONG_STEP;
      }
      else
         *Flags &= ~ESTATE_LONG_STEP;

      // set step endpoint
      *P = next->Meta->Location;

	  // cache hit
      return true;
   }
   
   // no cache hit
   return false;
}
#endif

bool AStarGetStepTowards(room_type* Room, V2* S, V2* E, V2* P, unsigned int* Flags, int ObjectID)
{
   // convert coordinates from ROO floatingpoint to
   // highres scale in integers
   int startrow = (int)roundf(S->Y / 256.0f);
   int startcol = (int)roundf(S->X / 256.0f);
   int endrow = (int)roundf(E->Y / 256.0f);
   int endcol = (int)roundf(E->X / 256.0f);

   // all 4 values must be within grid array bounds!
   if (startrow < 0 || startrow >= Room->rowshighres ||
       startcol < 0 || startcol >= Room->colshighres ||
       endrow < 0 || endrow >= Room->rowshighres ||
       endcol < 0 || endcol >= Room->colshighres)
         return false;

   /**********************************************************************/

   // set data on astar
   AStar.StartNode = &Room->Grid[startrow][startcol];
   AStar.EndNode = &Room->Grid[endrow][endcol];
   AStar.Room = Room;
   AStar.LastNode = NULL;
   AStar.ObjectID = ObjectID;
   AStar.HeapSize = 0;

   /**********************************************************************/
#if PATHCACHEENABLED
   // first try path cache lookup
   if (AStarGetStepFromCache(Room, AStar.StartNode, AStar.EndNode, P, Flags, ObjectID))
      return true;
#endif
   /**********************************************************************/

   // prepare non-persistent astar grid data memory
   ZeroMemory(AStar.NodesData, AStar.NodesDataSize);
   //ZeroMemory(AStar.NodesData, Room->rowshighres * Room->colshighres * sizeof(astar_node_data));

   /**********************************************************************/

   // mark nodes blocked by objects 
   AStarAddBlockers(&AStar);

   // insert startnode into heap-tree
   AStarHeapInsert(&AStar, AStar.StartNode);

   // the algorithm finishes if we either hit a node close enough to endnode
   // or if there is no more entries in the open list (unreachable)
   while (AStarProcessFirst(&AStar)) {}

   //AStarWriteGridToFile(Room);
   //AStarWriteHeapToFile(Room);

   /**********************************************************************/

   // unreachable
   if (!AStar.LastNode)
      return false;

#if PATHCACHEENABLED
   // check for resetting nextpathidx
   if (Room->NextPathIdx >= PATHCACHESIZE)
      Room->NextPathIdx = 0;

   // get path from cache and clear it
   astar_path* path = Room->Paths[Room->NextPathIdx];
   Room->NextPathIdx++;

   // clear old cached path
   path->clear();
#else
   // use a temporary ::std:list
   astar_path _path;
   astar_path* path = &_path;
#endif

   // walk back parent pointers from lastnode (=path)
   astar_node* node = AStar.LastNode;
   while (node)
   {
	   path->push_front(node);
	   node = node->Data->parent;
   }

   // not interested in first one (=startnode)
   path->pop_front();

   // this is the next one
   node = path->front();

   // for diagonal moves mark to be long step (required for timer elapse)
   if (abs(node->Meta->Col - AStar.StartNode->Meta->Col) &&
	   abs(node->Meta->Row - AStar.StartNode->Meta->Row))
   {
      *Flags |= ESTATE_LONG_STEP;
   }
   else
      *Flags &= ~ESTATE_LONG_STEP;

   // set step endpoint
   *P = node->Meta->Location;

   return true;
}

void AStarInit()
{
   // memory for astar data erased with each query
   AStar.NodesDataSize = NODESDATASQUARES * sizeof(astar_node_data);
   AStar.NodesData = (astar_node_data*)AllocateMemory(
	   MALLOC_ID_ASTAR, AStar.NodesDataSize);

   AStar.StartNode = NULL;
   AStar.EndNode   = NULL;
   AStar.LastNode  = NULL;
   AStar.ObjectID = 0;
   AStar.HeapSize = 0;    
   AStar.Room = NULL;

   // allocate memory for the persistent grid
   AStar.Grid = (astar_node_meta**)AllocateMemory(MALLOC_ID_ASTAR, MAXGRIDROWS * sizeof(astar_node_meta*));

   // allocate rows
   for (int i = 0; i < MAXGRIDROWS; i++)
	   AStar.Grid[i] = (astar_node_meta*)AllocateMemory(MALLOC_ID_ASTAR, MAXGRIDCOLS * sizeof(astar_node_meta));

   // setup all squares
   for (int i = 0; i < MAXGRIDROWS; i++)
   {
	   for (int j = 0; j < MAXGRIDCOLS; j++)
	   {
		   int idx = i*MAXGRIDCOLS + j;

		   astar_node_meta* node = &AStar.Grid[i][j];

		   // set row/col
		   node->Row = i;
		   node->Col = j;

		   // floatingpoint coordinates of the center of the square in ROO fineness (for queries)
		   node->Location.X = j * 256.0f;// +128.0f;
		   node->Location.Y = i * 256.0f;// +128.0f;
	   }
   }
}

void AStarShutdown()
{
   FreeMemory(MALLOC_ID_ASTAR, AStar.NodesData, AStar.NodesDataSize);
   
   // free each meta grid row
   for (int i = 0; i < MAXGRIDROWS; i++)
	   FreeMemory(MALLOC_ID_ASTAR, AStar.Grid[i], MAXGRIDCOLS * sizeof(astar_node_meta));

   // free meta grid
   FreeMemory(MALLOC_ID_ASTAR, AStar.Grid, MAXGRIDROWS * sizeof(astar_node_meta*));
}
#pragma endregion
