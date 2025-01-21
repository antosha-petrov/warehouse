using System.Collections.ObjectModel;
using WarehouseApp.Model;

namespace WarehouseApp.ViewModel
{
    public class OrderPageViewModel
    {
        public ObservableCollection<CartListItemViewModel> Items { get; }

        public OrderPageViewModel(IEnumerable<Component> components)
        {
            Items = new ObservableCollection<CartListItemViewModel>(
                components.Select(component => new CartListItemViewModel(component.Name, component))
            );
        }
    }
}
