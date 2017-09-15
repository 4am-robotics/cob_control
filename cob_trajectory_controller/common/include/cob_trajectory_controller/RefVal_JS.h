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


#ifndef _REFVAL_JS_H_
#define _REFVAL_JS_H_

#include <vector>


class RefVal_JS
{
	public:
		virtual std::vector<double> r(double s) const=0;
		virtual double s(double t) const=0;
		virtual std::vector<double> r_t(double t) const { return r( s(t) ); }

		virtual std::vector<double> dr_ds(double s) const=0;
		virtual double ds_dt(double t) const=0;
		virtual std::vector<double> dr_dt(double t) const
		{
			std::vector<double> dr;
			dr.resize(dr_ds(t).size());
			for(unsigned int i = 0; i < dr_ds(t).size(); i++)
				dr.at(i) = dr_ds( s(t) ).at(i) * ds_dt( t );
			return dr;
		}

		virtual std::vector<double> getLast() const { return r_t( getTotalTime() ); }

		virtual double getTotalTime() const=0;
};

#endif

