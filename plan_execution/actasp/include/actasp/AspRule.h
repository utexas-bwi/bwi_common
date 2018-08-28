#pragma once

#include <actasp/AspLaw.h>
#include <actasp/AspFluent.h>

namespace actasp {

//this used to be a separate class, but it's now a specialization of AspLaw
  
typedef AspLaw<AspFluent> AspRule;
	
}


