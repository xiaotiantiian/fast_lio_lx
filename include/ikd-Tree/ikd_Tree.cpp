#include "ikd_Tree.h"
#include <ros/ros.h>
/*
Description: ikd-Tree: an incremental k-d tree for robotic applications (single-threaded version)
Author: Yixi Cai
email: yixicai@connect.hku.hk
Modified: Multi-threading disabled for deterministic behavior
*/

template <typename PointType>
KD_TREE<PointType>::KD_TREE(float delete_param, float balance_param, float box_length)
{
    delete_criterion_param = delete_param;
    balance_criterion_param = balance_param;
    downsample_size = box_length;
    Root_Node = nullptr;
    STATIC_ROOT_NODE = nullptr;
    Delete_Storage_Disabled = false;
}

template <typename PointType>
KD_TREE<PointType>::~KD_TREE()
{
    Delete_Storage_Disabled = true;
    delete_tree_nodes(&Root_Node);
    PointVector().swap(PCL_Storage);
    PointVector().swap(Points_deleted);
}

template <typename PointType>
void KD_TREE<PointType>::Set_delete_criterion_param(float delete_param)
{
    delete_criterion_param = delete_param;
}

template <typename PointType>
void KD_TREE<PointType>::Set_balance_criterion_param(float balance_param)
{
    balance_criterion_param = balance_param;
}

template <typename PointType>
void KD_TREE<PointType>::set_downsample_param(float downsample_param)
{
    downsample_size = downsample_param;
}

template <typename PointType>
void KD_TREE<PointType>::InitializeKDTree(float delete_param, float balance_param, float box_length)
{
    Set_delete_criterion_param(delete_param);
    Set_balance_criterion_param(balance_param);
    set_downsample_param(box_length);
}

template <typename PointType>
void KD_TREE<PointType>::InitTreeNode(KD_TREE_NODE *root)
{
    root->point.x = 0.0f;
    root->point.y = 0.0f;
    root->point.z = 0.0f;
    root->node_range_x[0] = 0.0f;
    root->node_range_x[1] = 0.0f;
    root->node_range_y[0] = 0.0f;
    root->node_range_y[1] = 0.0f;
    root->node_range_z[0] = 0.0f;
    root->node_range_z[1] = 0.0f;
    root->division_axis = 0;
    root->father_ptr = nullptr;
    root->left_son_ptr = nullptr;
    root->right_son_ptr = nullptr;
    root->TreeSize = 0;
    root->invalid_point_num = 0;
    root->down_del_num = 0;
    root->point_deleted = false;
    root->tree_deleted = false;
    root->need_push_down_to_left = false;
    root->need_push_down_to_right = false;
    root->point_downsample_deleted = false;
    root->working_flag = false;
}

template <typename PointType>
int KD_TREE<PointType>::size()
{
    if (Root_Node != nullptr)
        return Root_Node->TreeSize;
    return 0;
}

template <typename PointType>
BoxPointType KD_TREE<PointType>::tree_range()
{
    BoxPointType range;
    if (Root_Node != nullptr)
    {
        range.vertex_min[0] = Root_Node->node_range_x[0];
        range.vertex_min[1] = Root_Node->node_range_y[0];
        range.vertex_min[2] = Root_Node->node_range_z[0];
        range.vertex_max[0] = Root_Node->node_range_x[1];
        range.vertex_max[1] = Root_Node->node_range_y[1];
        range.vertex_max[2] = Root_Node->node_range_z[1];
    }
    else
    {
        memset(&range, 0, sizeof(range));
    }
    return range;
}

template <typename PointType>
int KD_TREE<PointType>::validnum()
{
    if (Root_Node != nullptr)
        return (Root_Node->TreeSize - Root_Node->invalid_point_num);
    return 0;
}

template <typename PointType>
void KD_TREE<PointType>::root_alpha(float &alpha_bal, float &alpha_del)
{
    alpha_bal = Root_Node->alpha_bal;
    alpha_del = Root_Node->alpha_del;
}

template <typename PointType>
void KD_TREE<PointType>::Build(PointVector point_cloud)
{
    if (Root_Node != nullptr)
    {
        delete_tree_nodes(&Root_Node);
    }
    if (point_cloud.size() == 0)
        return;
    STATIC_ROOT_NODE = new KD_TREE_NODE;
    InitTreeNode(STATIC_ROOT_NODE);
    BuildTree(&STATIC_ROOT_NODE->left_son_ptr, 0, point_cloud.size() - 1, point_cloud);
    Update(STATIC_ROOT_NODE);
    STATIC_ROOT_NODE->TreeSize = 0;
    Root_Node = STATIC_ROOT_NODE->left_son_ptr;
}

template <typename PointType>
void KD_TREE<PointType>::Nearest_Search(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, double max_dist)
{
    MANUAL_HEAP q(2 * k_nearest);
    q.clear();
    Point_Distance.clear();
    Search(Root_Node, k_nearest, point, q, max_dist);
    int k_found = min(k_nearest, int(q.size()));
    Nearest_Points.clear();
    Point_Distance.clear();
    for (int i = 0; i < k_found; i++)
    {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
}

template <typename PointType>
void KD_TREE<PointType>::Box_Search(const BoxPointType &Box_of_Point, PointVector &Storage)
{
    Storage.clear();
    Search_by_range(Root_Node, Box_of_Point, Storage);
}

template <typename PointType>
void KD_TREE<PointType>::Radius_Search(PointType point, const float radius, PointVector &Storage)
{
    Storage.clear();
    Search_by_radius(Root_Node, point, radius, Storage);
}

template <typename PointType>
int KD_TREE<PointType>::Add_Points(PointVector &PointToAdd, bool downsample_on)
{
    std::sort(PointToAdd.begin(), PointToAdd.end(),
              [](const PointType &a, const PointType &b)
              {
                  if (a.x != b.x) return a.x < b.x;
                  if (a.y != b.y) return a.y < b.y;
                  return a.z < b.z;
              });
    int NewPointSize = PointToAdd.size();
    int tmp_counter = 0;
    BoxPointType Box_of_Point;
    PointType downsample_result, mid_point;
    bool downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
    float min_dist, tmp_dist;
    for (int i = 0; i < PointToAdd.size(); i++)
    {
        if (downsample_switch)
        {
            Box_of_Point.vertex_min[0] = floor(PointToAdd[i].x / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[0] = Box_of_Point.vertex_min[0] + downsample_size;
            Box_of_Point.vertex_min[1] = floor(PointToAdd[i].y / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[1] = Box_of_Point.vertex_min[1] + downsample_size;
            Box_of_Point.vertex_min[2] = floor(PointToAdd[i].z / downsample_size) * downsample_size;
            Box_of_Point.vertex_max[2] = Box_of_Point.vertex_min[2] + downsample_size;
            mid_point.x = Box_of_Point.vertex_min[0] + (Box_of_Point.vertex_max[0] - Box_of_Point.vertex_min[0]) / 2.0;
            mid_point.y = Box_of_Point.vertex_min[1] + (Box_of_Point.vertex_max[1] - Box_of_Point.vertex_min[1]) / 2.0;
            mid_point.z = Box_of_Point.vertex_min[2] + (Box_of_Point.vertex_max[2] - Box_of_Point.vertex_min[2]) / 2.0;
            Downsample_Storage.clear();
            Search_by_range(Root_Node, Box_of_Point, Downsample_Storage);
            min_dist = calc_dist(PointToAdd[i], mid_point);
            downsample_result = PointToAdd[i];
            for (int index = 0; index < Downsample_Storage.size(); index++)
            {
                tmp_dist = calc_dist(Downsample_Storage[index], mid_point);
                if (tmp_dist < min_dist)
                {
                    min_dist = tmp_dist;
                    downsample_result = Downsample_Storage[index];
                }
            }
            if (Downsample_Storage.size() > 1 || same_point(PointToAdd[i], downsample_result))
            {
                if (Downsample_Storage.size() > 0)
                    Delete_by_range(&Root_Node, Box_of_Point, true, true);
                Add_by_point(&Root_Node, downsample_result, true, Root_Node ? Root_Node->division_axis : 0);
                tmp_counter++;
            }
        }
        else
        {
            Add_by_point(&Root_Node, PointToAdd[i], true, Root_Node ? Root_Node->division_axis : 0);
        }
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Add_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        Add_by_range(&Root_Node, BoxPoints[i], true);
    }
}

template <typename PointType>
void KD_TREE<PointType>::Delete_Points(PointVector &PointToDel)
{
    for (int i = 0; i < PointToDel.size(); i++)
    {
        Delete_by_point(&Root_Node, PointToDel[i], true);
    }
}

template <typename PointType>
int KD_TREE<PointType>::Delete_Point_Boxes(vector<BoxPointType> &BoxPoints)
{
    int tmp_counter = 0;
    for (int i = 0; i < BoxPoints.size(); i++)
    {
        tmp_counter += Delete_by_range(&Root_Node, BoxPoints[i], true, false);
    }
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::acquire_removed_points(PointVector &removed_points)
{
    for (int i = 0; i < Points_deleted.size(); i++)
    {
        removed_points.push_back(Points_deleted[i]);
    }
    Points_deleted.clear();
}

template <typename PointType>
void KD_TREE<PointType>::BuildTree(KD_TREE_NODE **root, int l, int r, PointVector &Storage)
{
    if (l > r) return;
    *root = new KD_TREE_NODE;
    InitTreeNode(*root);
    int mid = (l + r) >> 1;
    int div_axis = 0;
    float min_value[3] = {INFINITY, INFINITY, INFINITY};
    float max_value[3] = {-INFINITY, -INFINITY, -INFINITY};
    float dim_range[3] = {0, 0, 0};
    for (int i = l; i <= r; i++)
    {
        min_value[0] = min(min_value[0], Storage[i].x);
        min_value[1] = min(min_value[1], Storage[i].y);
        min_value[2] = min(min_value[2], Storage[i].z);
        max_value[0] = max(max_value[0], Storage[i].x);
        max_value[1] = max(max_value[1], Storage[i].y);
        max_value[2] = max(max_value[2], Storage[i].z);
    }
    for (int i = 0; i < 3; i++)
        dim_range[i] = max_value[i] - min_value[i];
    for (int i = 1; i < 3; i++)
        if (dim_range[i] > dim_range[div_axis])
            div_axis = i;
    (*root)->division_axis = div_axis;
    switch (div_axis)
    {
    case 0:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    case 1:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_y);
        break;
    case 2:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_z);
        break;
    default:
        nth_element(begin(Storage) + l, begin(Storage) + mid, begin(Storage) + r + 1, point_cmp_x);
        break;
    }
    (*root)->point = Storage[mid];
    KD_TREE_NODE *left_son = nullptr, *right_son = nullptr;
    BuildTree(&left_son, l, mid - 1, Storage);
    BuildTree(&right_son, mid + 1, r, Storage);
    (*root)->left_son_ptr = left_son;
    (*root)->right_son_ptr = right_son;
    Update(*root);
}

template <typename PointType>
void KD_TREE<PointType>::Rebuild(KD_TREE_NODE **root)
{
    KD_TREE_NODE *father_ptr = (*root)->father_ptr;
    int size_rec = (*root)->TreeSize;
    PCL_Storage.clear();
    flatten(*root, PCL_Storage, DELETE_POINTS_REC);
    delete_tree_nodes(root);
    BuildTree(root, 0, PCL_Storage.size() - 1, PCL_Storage);
    if (*root != nullptr)
        (*root)->father_ptr = father_ptr;
    if (*root == Root_Node)
        STATIC_ROOT_NODE->left_son_ptr = *root;
}

template <typename PointType>
int KD_TREE<PointType>::Delete_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild, bool is_downsample)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return 0;
    (*root)->working_flag = true;
    Push_Down(*root);
    int tmp_counter = 0;
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return 0;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return 0;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return 0;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = true;
        (*root)->point_deleted = true;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        tmp_counter = (*root)->TreeSize - (*root)->invalid_point_num;
        (*root)->invalid_point_num = (*root)->TreeSize;
        if (is_downsample)
        {
            (*root)->tree_downsample_deleted = true;
            (*root)->point_downsample_deleted = true;
            (*root)->down_del_num = (*root)->TreeSize;
        }
        return tmp_counter;
    }
    if (!(*root)->point_deleted &&
        boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y &&
        boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = true;
        tmp_counter += 1;
        if (is_downsample)
            (*root)->point_downsample_deleted = true;
    }
    tmp_counter += Delete_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild, is_downsample);
    tmp_counter += Delete_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild, is_downsample);
    Update(*root);
    if (allow_rebuild && Criterion_Check(*root))
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
    return tmp_counter;
}

template <typename PointType>
void KD_TREE<PointType>::Delete_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild)
{
    if ((*root) == nullptr || (*root)->tree_deleted)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (same_point((*root)->point, point) && !(*root)->point_deleted)
    {
        (*root)->point_deleted = true;
        (*root)->invalid_point_num += 1;
        if ((*root)->invalid_point_num == (*root)->TreeSize)
            (*root)->tree_deleted = true;
        return;
    }
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) ||
        ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        Delete_by_point(&(*root)->left_son_ptr, point, allow_rebuild);
    }
    else
    {
        Delete_by_point(&(*root)->right_son_ptr, point, allow_rebuild);
    }
    Update(*root);
    if (allow_rebuild && Criterion_Check(*root))
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_range(KD_TREE_NODE **root, BoxPointType boxpoint, bool allow_rebuild)
{
    if ((*root) == nullptr)
        return;
    (*root)->working_flag = true;
    Push_Down(*root);
    if (boxpoint.vertex_max[0] <= (*root)->node_range_x[0] || boxpoint.vertex_min[0] > (*root)->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= (*root)->node_range_y[0] || boxpoint.vertex_min[1] > (*root)->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= (*root)->node_range_z[0] || boxpoint.vertex_min[2] > (*root)->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= (*root)->node_range_x[0] && boxpoint.vertex_max[0] > (*root)->node_range_x[1] &&
        boxpoint.vertex_min[1] <= (*root)->node_range_y[0] && boxpoint.vertex_max[1] > (*root)->node_range_y[1] &&
        boxpoint.vertex_min[2] <= (*root)->node_range_z[0] && boxpoint.vertex_max[2] > (*root)->node_range_z[1])
    {
        (*root)->tree_deleted = false || (*root)->tree_downsample_deleted;
        (*root)->point_deleted = false || (*root)->point_downsample_deleted;
        (*root)->need_push_down_to_left = true;
        (*root)->need_push_down_to_right = true;
        (*root)->invalid_point_num = (*root)->down_del_num;
        return;
    }
    if (boxpoint.vertex_min[0] <= (*root)->point.x && boxpoint.vertex_max[0] > (*root)->point.x &&
        boxpoint.vertex_min[1] <= (*root)->point.y && boxpoint.vertex_max[1] > (*root)->point.y &&
        boxpoint.vertex_min[2] <= (*root)->point.z && boxpoint.vertex_max[2] > (*root)->point.z)
    {
        (*root)->point_deleted = (*root)->point_downsample_deleted;
    }
    Add_by_range(&((*root)->left_son_ptr), boxpoint, allow_rebuild);
    Add_by_range(&((*root)->right_son_ptr), boxpoint, allow_rebuild);
    Update(*root);
    if (allow_rebuild && Criterion_Check(*root))
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
}

template <typename PointType>
void KD_TREE<PointType>::Add_by_point(KD_TREE_NODE **root, PointType point, bool allow_rebuild, int father_axis)
{
    if (*root == nullptr)
    {
        *root = new KD_TREE_NODE;
        InitTreeNode(*root);
        (*root)->point = point;
        (*root)->division_axis = (father_axis + 1) % 3;
        Update(*root);
        return;
    }
    (*root)->working_flag = true;
    Push_Down(*root);
    if (((*root)->division_axis == 0 && point.x < (*root)->point.x) ||
        ((*root)->division_axis == 1 && point.y < (*root)->point.y) ||
        ((*root)->division_axis == 2 && point.z < (*root)->point.z))
    {
        Add_by_point(&(*root)->left_son_ptr, point, allow_rebuild, (*root)->division_axis);
    }
    else
    {
        Add_by_point(&(*root)->right_son_ptr, point, allow_rebuild, (*root)->division_axis);
    }
    Update(*root);
    if (allow_rebuild && Criterion_Check(*root))
        Rebuild(root);
    if ((*root) != nullptr)
        (*root)->working_flag = false;
}

template <typename PointType>
void KD_TREE<PointType>::Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, double max_dist)
{
    if (root == nullptr || root->tree_deleted)
        return;
    double cur_dist = calc_box_dist(root, point);
    double max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr)
        return;
    Push_Down(root);
    if (!root->point_deleted)
    {
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist))
        {
            if (q.size() >= k_nearest)
                q.pop();
            PointType_CMP current_point{root->point, dist};
            q.push(current_point);
        }
    }
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist)
    {
        if (dist_left_node <= dist_right_node)
        {
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            if (q.size() < k_nearest || dist_right_node < q.top().dist)
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
        }
        else
        {
            Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            if (q.size() < k_nearest || dist_left_node < q.top().dist)
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
        }
    }
    else
    {
        if (dist_left_node < q.top().dist)
            Search(root->left_son_ptr, k_nearest, point, q, max_dist);
        if (dist_right_node < q.top().dist)
            Search(root->right_son_ptr, k_nearest, point, q, max_dist);
    }
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_range(KD_TREE_NODE *root, BoxPointType boxpoint, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (boxpoint.vertex_max[0] <= root->node_range_x[0] || boxpoint.vertex_min[0] > root->node_range_x[1])
        return;
    if (boxpoint.vertex_max[1] <= root->node_range_y[0] || boxpoint.vertex_min[1] > root->node_range_y[1])
        return;
    if (boxpoint.vertex_max[2] <= root->node_range_z[0] || boxpoint.vertex_min[2] > root->node_range_z[1])
        return;
    if (boxpoint.vertex_min[0] <= root->node_range_x[0] && boxpoint.vertex_max[0] > root->node_range_x[1] &&
        boxpoint.vertex_min[1] <= root->node_range_y[0] && boxpoint.vertex_max[1] > root->node_range_y[1] &&
        boxpoint.vertex_min[2] <= root->node_range_z[0] && boxpoint.vertex_max[2] > root->node_range_z[1])
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (boxpoint.vertex_min[0] <= root->point.x && boxpoint.vertex_max[0] > root->point.x &&
        boxpoint.vertex_min[1] <= root->point.y && boxpoint.vertex_max[1] > root->point.y &&
        boxpoint.vertex_min[2] <= root->point.z && boxpoint.vertex_max[2] > root->point.z)
    {
        if (!root->point_deleted)
            Storage.push_back(root->point);
    }
    Search_by_range(root->left_son_ptr, boxpoint, Storage);
    Search_by_range(root->right_son_ptr, boxpoint, Storage);
}

template <typename PointType>
void KD_TREE<PointType>::Search_by_radius(KD_TREE_NODE *root, PointType point, float radius, PointVector &Storage)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    PointType range_center;
    range_center.x = (root->node_range_x[0] + root->node_range_x[1]) * 0.5;
    range_center.y = (root->node_range_y[0] + root->node_range_y[1]) * 0.5;
    range_center.z = (root->node_range_z[0] + root->node_range_z[1]) * 0.5;
    float dist = sqrt(calc_dist(range_center, point));
    if (dist > radius + sqrt(root->radius_sq))
        return;
    if (dist <= radius - sqrt(root->radius_sq))
    {
        flatten(root, Storage, NOT_RECORD);
        return;
    }
    if (!root->point_deleted && calc_dist(root->point, point) <= radius * radius)
    {
        Storage.push_back(root->point);
    }
    Search_by_radius(root->left_son_ptr, point, radius, Storage);
    Search_by_radius(root->right_son_ptr, point, radius, Storage);
}

template <typename PointType>
bool KD_TREE<PointType>::Criterion_Check(KD_TREE_NODE *root)
{
    if (root->TreeSize <= Minimal_Unbalanced_Tree_Size)
        return false;
    float balance_evaluation = 0.0f;
    float delete_evaluation = 0.0f;
    KD_TREE_NODE *son_ptr = root->left_son_ptr;
    if (son_ptr == nullptr)
        son_ptr = root->right_son_ptr;
    delete_evaluation = float(root->invalid_point_num) / root->TreeSize;
    balance_evaluation = float(son_ptr->TreeSize) / (root->TreeSize - 1);
    if (delete_evaluation > delete_criterion_param)
        return true;
    if (balance_evaluation > balance_criterion_param || balance_evaluation < 1 - balance_criterion_param)
        return true;
    return false;
}

template <typename PointType>
void KD_TREE<PointType>::Push_Down(KD_TREE_NODE *root)
{
    if (root == nullptr)
        return;
    if (root->need_push_down_to_left && root->left_son_ptr != nullptr)
    {
        root->left_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
        root->left_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
        root->left_son_ptr->tree_deleted = root->tree_deleted || root->left_son_ptr->tree_downsample_deleted;
        root->left_son_ptr->point_deleted = root->left_son_ptr->tree_deleted || root->left_son_ptr->point_downsample_deleted;
        if (root->tree_downsample_deleted)
            root->left_son_ptr->down_del_num = root->left_son_ptr->TreeSize;
        if (root->tree_deleted)
            root->left_son_ptr->invalid_point_num = root->left_son_ptr->TreeSize;
        else
            root->left_son_ptr->invalid_point_num = root->left_son_ptr->down_del_num;
        root->left_son_ptr->need_push_down_to_left = true;
        root->left_son_ptr->need_push_down_to_right = true;
        root->need_push_down_to_left = false;
    }
    if (root->need_push_down_to_right && root->right_son_ptr != nullptr)
    {
        root->right_son_ptr->tree_downsample_deleted |= root->tree_downsample_deleted;
        root->right_son_ptr->point_downsample_deleted |= root->tree_downsample_deleted;
        root->right_son_ptr->tree_deleted = root->tree_deleted || root->right_son_ptr->tree_downsample_deleted;
        root->right_son_ptr->point_deleted = root->right_son_ptr->tree_deleted || root->right_son_ptr->point_downsample_deleted;
        if (root->tree_downsample_deleted)
            root->right_son_ptr->down_del_num = root->right_son_ptr->TreeSize;
        if (root->tree_deleted)
            root->right_son_ptr->invalid_point_num = root->right_son_ptr->TreeSize;
        else
            root->right_son_ptr->invalid_point_num = root->right_son_ptr->down_del_num;
        root->right_son_ptr->need_push_down_to_left = true;
        root->right_son_ptr->need_push_down_to_right = true;
        root->need_push_down_to_right = false;
    }
}

template <typename PointType>
void KD_TREE<PointType>::Update(KD_TREE_NODE *root)
{
    KD_TREE_NODE *left_son_ptr = root->left_son_ptr;
    KD_TREE_NODE *right_son_ptr = root->right_son_ptr;
    float tmp_range_x[2] = {INFINITY, -INFINITY};
    float tmp_range_y[2] = {INFINITY, -INFINITY};
    float tmp_range_z[2] = {INFINITY, -INFINITY};
    if (left_son_ptr != nullptr && right_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + right_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(min(left_son_ptr->node_range_x[0], right_son_ptr->node_range_x[0]), root->point.x);
            tmp_range_x[1] = max(max(left_son_ptr->node_range_x[1], right_son_ptr->node_range_x[1]), root->point.x);
            tmp_range_y[0] = min(min(left_son_ptr->node_range_y[0], right_son_ptr->node_range_y[0]), root->point.y);
            tmp_range_y[1] = max(max(left_son_ptr->node_range_y[1], right_son_ptr->node_range_y[1]), root->point.y);
            tmp_range_z[0] = min(min(left_son_ptr->node_range_z[0], right_son_ptr->node_range_z[0]), root->point.z);
            tmp_range_z[1] = max(max(left_son_ptr->node_range_z[1], right_son_ptr->node_range_z[1]), root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (left_son_ptr != nullptr)
    {
        root->TreeSize = left_son_ptr->TreeSize + 1;
        root->invalid_point_num = left_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = left_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = left_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = left_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!left_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(left_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(left_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(left_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(left_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(left_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(left_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!left_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], left_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], left_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], left_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], left_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], left_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], left_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else if (right_son_ptr != nullptr)
    {
        root->TreeSize = right_son_ptr->TreeSize + 1;
        root->invalid_point_num = right_son_ptr->invalid_point_num + (root->point_deleted ? 1 : 0);
        root->down_del_num = right_son_ptr->down_del_num + (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = right_son_ptr->tree_downsample_deleted & root->point_downsample_deleted;
        root->tree_deleted = right_son_ptr->tree_deleted && root->point_deleted;
        if (root->tree_deleted || (!right_son_ptr->tree_deleted && !root->point_deleted))
        {
            tmp_range_x[0] = min(right_son_ptr->node_range_x[0], root->point.x);
            tmp_range_x[1] = max(right_son_ptr->node_range_x[1], root->point.x);
            tmp_range_y[0] = min(right_son_ptr->node_range_y[0], root->point.y);
            tmp_range_y[1] = max(right_son_ptr->node_range_y[1], root->point.y);
            tmp_range_z[0] = min(right_son_ptr->node_range_z[0], root->point.z);
            tmp_range_z[1] = max(right_son_ptr->node_range_z[1], root->point.z);
        }
        else
        {
            if (!right_son_ptr->tree_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], right_son_ptr->node_range_x[0]);
                tmp_range_x[1] = max(tmp_range_x[1], right_son_ptr->node_range_x[1]);
                tmp_range_y[0] = min(tmp_range_y[0], right_son_ptr->node_range_y[0]);
                tmp_range_y[1] = max(tmp_range_y[1], right_son_ptr->node_range_y[1]);
                tmp_range_z[0] = min(tmp_range_z[0], right_son_ptr->node_range_z[0]);
                tmp_range_z[1] = max(tmp_range_z[1], right_son_ptr->node_range_z[1]);
            }
            if (!root->point_deleted)
            {
                tmp_range_x[0] = min(tmp_range_x[0], root->point.x);
                tmp_range_x[1] = max(tmp_range_x[1], root->point.x);
                tmp_range_y[0] = min(tmp_range_y[0], root->point.y);
                tmp_range_y[1] = max(tmp_range_y[1], root->point.y);
                tmp_range_z[0] = min(tmp_range_z[0], root->point.z);
                tmp_range_z[1] = max(tmp_range_z[1], root->point.z);
            }
        }
    }
    else
    {
        root->TreeSize = 1;
        root->invalid_point_num = (root->point_deleted ? 1 : 0);
        root->down_del_num = (root->point_downsample_deleted ? 1 : 0);
        root->tree_downsample_deleted = root->point_downsample_deleted;
        root->tree_deleted = root->point_deleted;
        tmp_range_x[0] = root->point.x;
        tmp_range_x[1] = root->point.x;
        tmp_range_y[0] = root->point.y;
        tmp_range_y[1] = root->point.y;
        tmp_range_z[0] = root->point.z;
        tmp_range_z[1] = root->point.z;
    }
    memcpy(root->node_range_x, tmp_range_x, sizeof(tmp_range_x));
    memcpy(root->node_range_y, tmp_range_y, sizeof(tmp_range_y));
    memcpy(root->node_range_z, tmp_range_z, sizeof(tmp_range_z));
    float x_L = (root->node_range_x[1] - root->node_range_x[0]) * 0.5;
    float y_L = (root->node_range_y[1] - root->node_range_y[0]) * 0.5;
    float z_L = (root->node_range_z[1] - root->node_range_z[0]) * 0.5;
    root->radius_sq = x_L * x_L + y_L * y_L + z_L * z_L;
    if (left_son_ptr != nullptr)
        left_son_ptr->father_ptr = root;
    if (right_son_ptr != nullptr)
        right_son_ptr->father_ptr = root;
    if (root == Root_Node && root->TreeSize > 3)
    {
        KD_TREE_NODE *son_ptr = root->left_son_ptr;
        if (son_ptr == nullptr)
            son_ptr = root->right_son_ptr;
        float tmp_bal = float(son_ptr->TreeSize) / (root->TreeSize - 1);
        root->alpha_del = float(root->invalid_point_num) / root->TreeSize;
        root->alpha_bal = (tmp_bal >= 0.5 - EPSS) ? tmp_bal : 1.0f - tmp_bal;
    }
}

template <typename PointType>
void KD_TREE<PointType>::flatten(KD_TREE_NODE *root, PointVector &Storage, delete_point_storage_set storage_type)
{
    if (root == nullptr)
        return;
    Push_Down(root);
    if (!root->point_deleted)
        Storage.push_back(root->point);
    flatten(root->left_son_ptr, Storage, storage_type);
    flatten(root->right_son_ptr, Storage, storage_type);
    if (storage_type == DELETE_POINTS_REC)
    {
        if (root->point_deleted && !root->point_downsample_deleted)
        {
            Points_deleted.push_back(root->point);
        }
    }
}

template <typename PointType>
void KD_TREE<PointType>::delete_tree_nodes(KD_TREE_NODE **root)
{
    if (*root == nullptr)
        return;
    Push_Down(*root);
    delete_tree_nodes(&(*root)->left_son_ptr);
    delete_tree_nodes(&(*root)->right_son_ptr);
    delete *root;
    *root = nullptr;
}

template <typename PointType>
bool KD_TREE<PointType>::same_point(PointType a, PointType b)
{
    return (fabs(a.x - b.x) < EPSS && fabs(a.y - b.y) < EPSS && fabs(a.z - b.z) < EPSS);
}

template <typename PointType>
float KD_TREE<PointType>::calc_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    return dist;
}

template <typename PointType>
float KD_TREE<PointType>::calc_box_dist(KD_TREE_NODE *node, PointType point)
{
    if (node == nullptr)
        return INFINITY;
    float min_dist = 0.0;
    if (point.x < node->node_range_x[0])
        min_dist += (point.x - node->node_range_x[0]) * (point.x - node->node_range_x[0]);
    if (point.x > node->node_range_x[1])
        min_dist += (point.x - node->node_range_x[1]) * (point.x - node->node_range_x[1]);
    if (point.y < node->node_range_y[0])
        min_dist += (point.y - node->node_range_y[0]) * (point.y - node->node_range_y[0]);
    if (point.y > node->node_range_y[1])
        min_dist += (point.y - node->node_range_y[1]) * (point.y - node->node_range_y[1]);
    if (point.z < node->node_range_z[0])
        min_dist += (point.z - node->node_range_z[0]) * (point.z - node->node_range_z[0]);
    if (point.z > node->node_range_z[1])
        min_dist += (point.z - node->node_range_z[1]) * (point.z - node->node_range_z[1]);
    return min_dist;
}

template <typename PointType>
bool KD_TREE<PointType>::point_cmp_x(PointType a, PointType b) { return a.x < b.x; }
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_y(PointType a, PointType b) { return a.y < b.y; }
template <typename PointType>
bool KD_TREE<PointType>::point_cmp_z(PointType a, PointType b) { return a.z < b.z; }

template <typename T>
void MANUAL_Q<T>::clear()
{
    head = 0;
    tail = 0;
    counter = 0;
    is_empty = true;
}

template <typename T>
void MANUAL_Q<T>::pop()
{
    if (counter == 0)
        return;
    head++;
    head %= Q_LEN;
    counter--;
    if (counter == 0)
        is_empty = true;
}

template <typename T>
T MANUAL_Q<T>::front()
{
    return q[head];
}

template <typename T>
T MANUAL_Q<T>::back()
{
    return q[tail];
}

template <typename T>
void MANUAL_Q<T>::push(T op)
{
    q[tail] = op;
    counter++;
    if (is_empty)
        is_empty = false;
    tail++;
    tail %= Q_LEN;
}

template <typename T>
bool MANUAL_Q<T>::empty()
{
    return is_empty;
}

template <typename T>
int MANUAL_Q<T>::size()
{
    return counter;
}

template class KD_TREE<ikdTree_PointType>;
template class KD_TREE<pcl::PointXYZ>;
template class KD_TREE<pcl::PointXYZI>;
template class KD_TREE<pcl::PointXYZINormal>;