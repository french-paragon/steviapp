#include "dataexception.h"

#include "datablocks/project.h"

namespace StereoVisionApp {

DataException::DataException(QString const& what, const QObject *block):
	_block(block)
{
	std::string w = what.toStdString();
	_what = new char[w.size()];
	memcpy (_what, w.c_str(), w.size());
}


DataException::DataException(DataException const& other) :
	DataException(other.what(), other._block)
{

}

DataException::~DataException() {
	delete _what;
}

const char* DataException::what() const noexcept
{
	return _what;
}

const DataBlock *DataException::block() const
{
	return qobject_cast<const DataBlock*>(_block);
}

const Project* DataException::project() const
{
	return qobject_cast<const Project*>(_block);
}

} // namespace StereoVisionApp
