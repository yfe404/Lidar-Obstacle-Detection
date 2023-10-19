
template<typename PointT>
struct Node
{
	PointT point;
	int id;
	Node* left;
	Node* right;

	Node(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};


template<typename PointT>
struct KdTree
{
	Node<PointT>* root;

	KdTree()
	: root(NULL)
	{}

int getCoordinateValue(const PointT &point, uint cd) {
    if (cd == 0) return point.x;
    if (cd == 1) return point.y;
    if (cd == 2) return point.z; 
    return -1;  // Return some default or error value
}
  
void _insert(Node<PointT> **current, PointT data, int index, int depth) {
  if (*current == nullptr) {
    Node<PointT> * newNode = new Node<PointT>(data, index);
    *current = newNode;
  } else {
    uint cd = depth % 3;
    
    if (getCoordinateValue(data, cd) < getCoordinateValue((*current)->point, cd)) {
      this->_insert(&((*current)->left), data, index, depth+1);
    } else {
      this->_insert(&((*current)->right), data, index, depth+1);
    }
    
  }
}
  
  void insert(PointT point, int id)
  {
    if (this->root == nullptr) {
      Node<PointT> * newNode = new Node<PointT>(point, id);
      this->root = newNode;
    } else {
      this->_insert(&this->root, point, id, 0);
    }
  }
  
  void _search(PointT target, Node<PointT>* node, int depth, float distanceTol, std::vector<int>& ids){
    if (node != nullptr) {
      bool predicate1 = (node->point.x >= (target.x - distanceTol)) && (node->point.x <= (target.x + distanceTol)) ;
      bool predicate2 = (node->point.y >= (target.y - distanceTol)) && (node->point.y <= (target.y + distanceTol)) ;
      bool predicate3 = (node->point.z >= (target.z - distanceTol)) && (node->point.z <= (target.z + distanceTol)) ;
      
      if (predicate1 && predicate2 && predicate3) {
	float deltaX = node->point.x - target.x;
	float deltaY = node->point.y - target.y;
	float deltaZ = node->point.z - target.z;
	float distance = sqrt(deltaX*deltaX + deltaY*deltaY + deltaZ*deltaZ);
	if (distance <= distanceTol)
	  ids.push_back(node->id);
      }

      if ( (getCoordinateValue(target, depth%3) - distanceTol) < getCoordinateValue(node->point, depth%3) )
	_search(target, node->left, depth+1, distanceTol, ids);
      if ( (getCoordinateValue(target, depth%3) + distanceTol) > getCoordinateValue(node->point, depth%3) )
	_search(target, node->right, depth+1, distanceTol, ids);
    }
  }
  
  
  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(PointT target, float distanceTol)
  {
    std::vector<int> ids;
    this->_search(target, this->root, 0, distanceTol, ids);
    return ids;
  }
};
