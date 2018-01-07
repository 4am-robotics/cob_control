/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef _REFVALSJS_PTP_H_
#define _REFVALSJS_PTP_H_

#include "RefVal_JS.h"
#include <cmath>
#include <iostream>


class RefValJS_PTP : public RefVal_JS
{
	public:
		RefValJS_PTP(const std::vector<double>& start, const std::vector<double>& ziel, double v_rad_s, double a_rad_s2);

		virtual std::vector<double> r(double s) const;
		virtual double s(double t) const;

		virtual std::vector<double> dr_ds(double s) const;
		virtual double ds_dt(double t) const;

		double getTotalTime() const { std::cout << m_T1 << "\n"; return m_T1 + m_T2 + m_T3; }

	protected:
		double norm(const std::vector<double>& j);
		double norm_max(const std::vector<double>& j);
		double norm_sqr(const std::vector<double>& j);
		double norm_weighted(const std::vector<double>& j);

		std::vector<double> m_start;
		std::vector<double> m_ziel;
		std::vector<double> m_direction;

		double m_length;

		double m_v_rad_s;
		double m_a_rad_s2;

		double m_T1;	// Dauer der Phase konst. Beschl.
		double m_T2;	// Dauer der Phase konst. Geschw.
		double m_T3;	// Dauer der Phase konst. Verzög.

		double m_sa1;	// "Beschl." des Wegparameters s in Phase 1
		double m_sv2;	// "Geschw." des Wegparameters s in Phase 2
		double m_sa3;	// "Verzög." des Wegparameters s in Phase 3

		static const double weigths[];

};

inline double RefValJS_PTP::norm(const std::vector<double>& j)
{
	// which norm should be used?
	return norm_weighted(j);
}

inline double RefValJS_PTP::norm_max(const std::vector<double>& j)
{
	double max = j.at(0);
	for	(unsigned int i = 0; i<j.size(); i++)
	{
		if(j.at(i) < max)
			max = j.at(i);
	}
	return max;
}

inline double RefValJS_PTP::norm_sqr(const std::vector<double>& j)
{
	double l = 0;
	for (unsigned int i = 0; i < j.size(); i++)
	{
		l += j[i] * j[i];
	}
	return sqrt(l);
}

inline double RefValJS_PTP::norm_weighted(const std::vector<double>& j)
{
	double l = 0;
	if ( j.size() == 7 )
	{
		for (unsigned int i = 0; i < j.size(); i++)
		{
			l += j[i]* weigths[i] * j[i] * weigths[i];
		}
	}
	else
	{
		for (unsigned int i = 0; i < j.size(); i++)
		{
			l += j[i] * j[i];
		}
	}
	return sqrt(l);
}


#endif

