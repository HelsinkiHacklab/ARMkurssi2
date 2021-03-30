/*
 * menu.h
 *
 *  Created on: Mar 17, 2021
 *      Author: Kremmen
 *
 *  Simple menu class that helps create basic selection based user interfaces for demos
 *  NOTE: This implementation uses dynamic allocation
 *  NOTE: no bounds checking is made against overallocating memory
 */

#ifndef SRC_MENU_H_
#define SRC_MENU_H_

#include <stdint.h>
#include <stdio.h>

#include <vector>

namespace CLIMenu {

typedef void menuItemHandler(char _selector );

struct menuItem {
	menuItem( char _selector, char *_prompt, menuItemHandler *_handler ) : selector(_selector), prompt(_prompt), handler(_handler) {};
	char selector;
	char *prompt;
	menuItemHandler *handler;
};

class menu {
public:
	menu( char *_header );
	virtual ~menu();
	void addItem( char _selector, char *_prompt, menuItemHandler *_handler );
	void addItem(menuItem *_item);
	void run( bool looping );
private:
	char * header;
	std::vector<menuItem *> menuItems;
};

} //namespace

#endif /* SRC_MENU_H_ */
