#include "cacheddistmap.h"






CachedDistanceMap::CachedDistanceMap(double scale, double max_dist,int size_x, int size_y) : 
  distances_(NULL), scale_(scale), max_dist_(max_dist), size_x_(size_x), size_y_(size_y)
{
  cell_radius_ = max_dist / scale;
  distances_ = new double *[cell_radius_+2];
  for(int i=0; i<=cell_radius_+1; i++)
    {
	    distances_[i] = new double[cell_radius_+2];
      for(int j=0; j<=cell_radius_+1; j++)
	    {
	      distances_[i][j] = sqrt(i*i + j*j);
	    }
    }
    cmap.name = "distance";
}


CachedDistanceMap::~CachedDistanceMap()
{
  if(distances_)
  {
	  for(int i=0; i<=cell_radius_+1; i++)
	    delete[] distances_[i];
	  delete[] distances_;
  }
}




//probably won't need
CachedDistanceMap* CachedDistanceMap::get_distance_map(double scale, double max_dist)
{
  static CachedDistanceMap* cdm = NULL;

  if(!cdm || (cdm->scale_ != scale) || (cdm->max_dist_ != max_dist))
  {
    if(cdm)
      delete cdm;
    cdm = new CachedDistanceMap(scale, max_dist, size_x_, size_y_);
  }

  return cdm;
}


void CachedDistanceMap::enqueue(layer_t occ_layer, int i, int j, int src_i, int src_j, std::priority_queue<CellData>& Q,
	     CachedDistanceMap* cdm, unsigned char* marked)
{


}

// Update the cspace distance values
void CachedDistanceMap::map_update_cspace(layer_t occ_layer,  int size_x, int size_y, double scale, double max_occ_dist)
{

  unsigned char* marked;
  std::priority_queue<CellData> Q;

  marked = new unsigned char[size_x*size_y];
  memset(marked, 0, sizeof(unsigned char) * size_x*size_y);

  //map->max_occ_dist = max_occ_dist;

  CachedDistanceMap* cdm = get_distance_map(scale, max_occ_dist);

// What to do for cell stuff???

  // Enqueue all the obstacle cells
  CellData cell;
  cell.layer = occ_layer;
  for(int i=0; i<size_x; i++)
  {
    cell.src_i_ = cell.i_ = i;
    for(int j=0; j<size_y; j++)
    {
      if(occ_layer.data[MapToDataIndex(i, j)]== +1)
      {
	//map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
        cmap.ddata[MapToDataIndex(i, j)] = 0.0;
	      cell.src_j_ = cell.j_ = j;
      	//marked[MAP_INDEX(map, i, j)] = 1;
        marked[MapToDataIndex(i, j)] = 1;
	      Q.push(cell);
      }
      else
        cmap.ddata[MapToDataIndex(i, j)] = max_occ_dist;
    }
  }

  while(!Q.empty())
  {
    CellData current_cell = Q.top();
    if(current_cell.i_ > 0)
      enqueue(occ_layer, current_cell.i_-1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if(current_cell.j_ > 0)
      enqueue(occ_layer, current_cell.i_, current_cell.j_-1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.i_ < size_x - 1)
      enqueue(occ_layer, current_cell.i_+1, current_cell.j_, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);
    if((int)current_cell.j_ < size_y - 1)
      enqueue(occ_layer, current_cell.i_, current_cell.j_+1, 
	      current_cell.src_i_, current_cell.src_j_,
	      Q, cdm, marked);

    Q.pop();
  }

  delete[] marked;


}

/*
Goal: create data layer for cspace distance values
Input: map object OR just meta info
Output: signed char * data
*/