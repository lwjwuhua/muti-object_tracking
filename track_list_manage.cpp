#include "track_list_manage.h"
#include"measurement_package.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

Track_manager::Track_manager()
{
	id = 0;
}

Track_manager::~Track_manager()
{
}

void Track_manager::Addtrack_list (const int& i_d, const VectorXd& m, const MatrixXd& P) {

	  id = i_d;
	  
	  track_mana.id = i_d-1;
	  track_mana.slot = 0;
	 track_mana.sig_track_state_list = VectorXd(4) ;
	 track_mana.sig_track_state_list<< m,0,0;

	 track_mana.sig_track_cov_list = P;
	 trackwith_id.push_back(track_mana);
	 
	//  measwith_id.erase();

	cout << "id =" << id << endl;
}



int Track_manager :: findelement(vector <Track_list_manag> v_, const int& key) {

	int len = v_.size();
	for ( inde = 0; inde < len; inde++)
	{
		if (v_.at(inde).id == key)
		{
			return inde;

		}

	}
	
	return -1;
}


void Track_manager::Deletrack_list( const int& id_) {

	trackwith_id.erase(std::begin(trackwith_id)+id_);


}

//    track_manager.measwith_id.erase(std::begin(track_manager.measwith_id) + 2);

	 //   track_manager.findelement(track_manager.measwith_id,7);


