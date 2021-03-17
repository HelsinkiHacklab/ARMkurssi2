/*
 * menu.cpp
 *
 *  Created on: Mar 17, 2021
 *      Author: Kremmen
 */


#include "menu.h"

extern "C" {
int _read(int fd, char *ptr, int len);
}

namespace CLIMenu {

menu::menu( char *_header ) {
	header = _header;
}

menu::~menu() {
	// TODO Auto-generated destructor stub
}

void menu::addItem( char _selector, char *_prompt, menuItemHandler *_handler ) {
	menuItems.push_back( new menuItem(_selector, _prompt, _handler) );
}

void menu::addItem(menuItem *_item) {
	menuItems.push_back( _item );
}

void menu::run( bool looping ) {
	menuItem *item;
	char _cmd;
	bool menuHit;

	while ( 1 ) {
		printf("\r\n%s\r\n", header);
		for ( uint8_t i = 0; i < menuItems.size(); i++ ) {
			item = menuItems.at(i);
			printf("%c: %s\r\n", item->selector, item->prompt);
		}
		printf("\r\n> ");
		fflush(stdout);
		_read(0, &_cmd, 1);
		if ( _cmd == 'x' ) return;
		menuHit = false;
		printf("  ");
		fflush(stdout);
		for ( uint8_t i = 0; i < menuItems.size(); i++ ) {
			item = menuItems.at(i);
			if ( item->selector == _cmd ) {
				menuHit = true;
				if ( item->handler != NULL ) {
					item->handler(_cmd);
					if ( !looping ) return;
				}
				else printf("  ... No associated action with selection!?\r\n");
			}
		}
		if ( !menuHit ) printf("  ... That did not compute???\r\n");
	}
}

} // namespace
