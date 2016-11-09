#include <stdio.h>
#include "FileUtils.h"

namespace AaltoGames
{

bool fileExists(const char *fileName)
{
	FILE *f=fopen(fileName,"rb");
	if (f!=NULL)
	{
		fclose(f);
		return true;
	}
	return false;
}


}