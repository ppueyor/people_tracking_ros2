
#include <PeopleDetector.h>
#include <ostream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <list>

using namespace std;

//-----------------------------------------------------
//
//-----------------------------------------------------
PeopleDetector::PeopleDetector() 
{
	weak_classifiers = NULL;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
PeopleDetector::~PeopleDetector() 
{
	if ( weak_classifiers != NULL ) {	
		weak_classifiers->setAutoDelete(true);
		delete weak_classifiers;
		weak_classifiers = NULL;
	}
}


//-----------------------------------------------------
// Load Hypotheses from a file
//-----------------------------------------------------
void PeopleDetector::load(FILE *f, int num) 
{
	WeakHypothesis *w;

	weak_classifiers = new dyntab_hypotheses(num);

	for(int i=0; i<num; i++) {
		w = new WeakHypothesis;	 
		w->load(f);
		weak_classifiers->add(w);
	}
}


//-----------------------------------------------------
//
// ----------------------------------------------------
int PeopleDetector::classify(Segment *s) 
{
	WeakHypothesis *w;
	double sum = 0.0;
	int type;
	double w_type;

	int n = weak_classifiers->num();

	for(int i=0; i<n; i++) {
		w = weak_classifiers->getElement(i);
		w_type = w->classify(s);
		sum = sum + w->alpha * w_type;
	}	
	
	if ( sum < 0.0 ) {
		type = -1;
	}
	else {
		type = 1;
	}

	s->type = type;

	return type;	
}


//-----------------------------------------------------
// segment a scan
// ----------------------------------------------------
//#define  _SEGMENT_SCAN_OLD_VERSION_

#ifdef _SEGMENT_SCAN_OLD_VERSION_
void PeopleDetector::segmentScan(double threshold, int n_readings, double *angles, double *ranges, dyntab_segments *segments)
{
	//cerr << "PeopleDetector::segmentScan" << endl; 
	Segment *s;
	Beam *b=NULL, *b_prev=NULL;	
	s = new Segment;
	//start with a segment
	for (int i=0; i<n_readings; i++) {			
		// read the first point
		b = new Beam;
		b->range = ranges[i];
		b->angle = angles[i];
		b->num_readings = n_readings;
		b->position = i;					
		if ( s->num() > 0 ) {
			if ( fabs(b_prev->range - b->range) < threshold ) {
				// same segment
				b_prev = b;
				b->segment_id = segments->num(); 
				s->add(b);					
			}
			else {
				segments->add(s);
				s = new Segment;
				b->segment_id = segments->num(); 
				s->add(b);
				b_prev = b;
			} 
		}	
		else {
			b_prev = b;
			s->add(b);	
		}						
	}
	segments->add(s);
} 
#else
void PeopleDetector::segmentScan(double threshold, int n_readings, double *angles, double *ranges, dyntab_segments *segments)
{

	//cerr << "PeopleDetector::segmentScan" << endl; 

	Segment *s=NULL;
	Beam *b=NULL; 
	std::list<Beam*> b_prev;
	bool new_segment;
	for (int i=0; i<n_readings; i++) {
				
		// read the first point
		b = new Beam;
		b->range = ranges[i];
		b->angle = angles[i];
		b->num_readings = n_readings;
		b->position = i;
		new_segment=true;
		for (std::list<Beam*>::iterator it = b_prev.begin(); it!= b_prev.end(); ++it) {
			if (fabs((*it)->range - b->range) < threshold) {
				new_segment=false;
				break;
			}
		}
		if (new_segment) {
			if (s!=NULL && s->num()>4) {
				segments->add(s);
			}
			s = new Segment;
		}
		b->segment_id = segments->num(); 	
		s->add(b);
		b_prev.push_back(b);
		if (b_prev.size()>3) {
			b_prev.pop_front();
		}						
	}
	if (s!=NULL) {
		segments->add(s);
	}
} 



#endif























