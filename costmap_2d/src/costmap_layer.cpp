#include<costmap_2d/costmap_layer.h>

namespace costmap_2d
{

// 包含进 （x,y），更新 bounding box
void CostmapLayer::touch(double x, double y, double* min_x, double* min_y, double* max_x, double* max_y)
{
    *min_x = std::min(x, *min_x);
    *min_y = std::min(y, *min_y);
    *max_x = std::max(x, *max_x);
    *max_y = std::max(y, *max_y);
}

// 是根据 Layer 中的 LayeredCostmap 更新 Costmap2D
void CostmapLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();		// layered_costmap_ 是 Layer 类中的成员变量
// resizeMap 是 Costmap2D 类中的函数，因为 CostmapLayer 类共有继承了 Costmap2D,所以可以直接使用这个函数根据 layered_costmap 中的costmap初始化 公有继承的 Costmap2D 类
// 相当于把 Costmap2D 类中 除了 默认值 以外的东西全部改变
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}

// 求两片区域的最小并集，更新 bounding box
void CostmapLayer::addExtraBounds(double mx0, double my0, double mx1, double my1)
{
    extra_min_x_ = std::min(mx0, extra_min_x_);
    extra_max_x_ = std::max(mx1, extra_max_x_);
    extra_min_y_ = std::min(my0, extra_min_y_);
    extra_max_y_ = std::max(my1, extra_max_y_);
    has_extra_bounds_ = true;
}

// 更新参数中指定的 bounding box 以包含来自 addExtraBounds 中的 bounding box
// 4个参数既是输入量也是输出量
void CostmapLayer::useExtraBounds(double* min_x, double* min_y, double* max_x, double* max_y)
{
    if (!has_extra_bounds_)				// 如果没有 extra_bounds 就返回
        return;

    *min_x = std::min(extra_min_x_, *min_x);		// 相当于取两个集合的并集，再把 extra_bounds 重置
    *min_y = std::min(extra_min_y_, *min_y);
    *max_x = std::max(extra_max_x_, *max_x);
    *max_y = std::max(extra_max_y_, *max_y);
    extra_min_x_ = 1e6;
    extra_min_y_ = 1e6;
    extra_max_x_ = -1e6;
    extra_max_y_ = -1e6;
    has_extra_bounds_ = false;
}

// 参数中给定的 master_grid 和 成员变量 costmap_ 的大小最好要相同
// 使用相应区域中较大的值覆盖 master_array , 如果它原来是 NO_INFORMATION 则也被覆盖
// 如果 costmap_ 是  NO_INFORMATION 则不进行操作
void CostmapLayer::updateWithMax(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)			// enabled_ 是 Layer 类中的对象，如果 enabled_ = false 就返回
    return;

  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();		// return size_x_

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)				// 对于参数中规定的坐标范围内
    {
// 这个 costmap_ 是哪儿的？ 这是 Costmap_2D 类中的 unsigned char* costmap_
      if (costmap_[it] == NO_INFORMATION){			// 如果 cost = NO_INFORMATION 就不做修改
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION || old_cost < costmap_[it])
        master_array[it] = costmap_[it];
      it++;
    }
  }
}

// 使用当前层的cost值更新 master_grid 指定边界框中的值
void CostmapLayer::updateWithTrueOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
                                           int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      master[it] = costmap_[it];
      it++;
    }
  }
}

// 使用当前层的cost值更新 master_grid 指定边界框中的值
// 如果当前层的 cost 值 = NO_INFORMATION 就不进行拷贝
void CostmapLayer::updateWithOverwrite(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = span*j+min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] != NO_INFORMATION)
        master[it] = costmap_[it];
      it++;
    }
  }
}

// 使用当前层的cost值更新 master_grid 指定边界框中的值
// 设置新的值为 master_grid 和 当前层中值 的和
// 如果 master_grid 是 NO_INFORMATION 那么被重写
// 如果当前层是 NO_INFORMATION ， 那么不进行操作
void CostmapLayer::updateWithAddition(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int span = master_grid.getSizeInCellsX();

  for (int j = min_j; j < max_j; j++)
  {
    unsigned int it = j * span + min_i;
    for (int i = min_i; i < max_i; i++)
    {
      if (costmap_[it] == NO_INFORMATION){
        it++;
        continue;
      }

      unsigned char old_cost = master_array[it];
      if (old_cost == NO_INFORMATION)
        master_array[it] = costmap_[it];
      else
      {
        int sum = old_cost + costmap_[it];
        if (sum >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            master_array[it] = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;
        else
            master_array[it] = sum;
      }
      it++;
    }
  }
}
}  // namespace costmap_2d
