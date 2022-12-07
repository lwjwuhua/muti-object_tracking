#ifndef TRACK_LIST_MANAGER_H_
#define TRACK_LIST_MANAGER_H_
#include "Dense"
#include <vector>
#include"measurement_package.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
class Track_manager
{
public:
	Track_manager();
	virtual ~Track_manager();

	void Addtrack_list(const int& i_d,const VectorXd& m, const MatrixXd& P);
	int id ;

	void Deletrack_list( const int&id_ );
   
	Track_list_manag  track_mana;
	vector <Track_list_manag> trackwith_id;
	size_t inde;

	int findelement ( vector<Track_list_manag> v_, const int& key);


private:

};




#endif  //
